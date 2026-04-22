// =============================================================================
// Kapalı Çevrim Su Seviyesi Kontrol Sistemi (v1.0 - Endüstriyel Sürüm)
// Mimari: Olay-Güdümlü Durum Makinesi (Event-Driven State Machine)
// İşlemci: ATmega328P (16 MHz) / 2KB SRAM Optimizasyonlu
// =============================================================================

#include <Adafruit_GFX.h>    // Grafik temeli (Adafruit)
#include <Adafruit_ST7735.h> // ST7735S TFT sürücüsü
#include <Arduino.h>
#include <Keypad.h>  // 4x4 matris tuş takımı
#include <SPI.h>     // ST7735 SPI veriyolu
#include <avr/wdt.h> // Hardware Watchdog Timer
#include <stdlib.h>  // atoi() ve bellek fonksiyonları

// =============================================================================
// --- DONANIM SABİTLERİ (HAL Katmanı) ---
// =============================================================================
#define TRIGGER_PIN 3          // HC-SR04 Trigger → D3
#define ECHO_PIN 2             // HC-SR04 Echo    → D2 (INT0)
#define DOLUM_POMPASI_PIN 5    // Dolum Pompası   → D5 (Timer0 PWM)
#define BOSALTIM_POMPASI_PIN 6 // Boşaltım Pompası→ D6 (Timer0 PWM)

#define OLUBANT_ESIGI 40 // Bu PWM'in altında motorlar dönmez
#define PWM_MAX 255      // analogWrite maksimum sınırı
// NOT: Eski float INTEGRAL_LIMIT kaldırıldı. Tek kaynak: INTEGRAL_LIMIT_Q8 (FAZ
// 4)
#define STALL_TOLERANCE 3     // mm — stall deadband (sensör jitterını emer)
#define SENSOR_TIMEOUT_MS 500 // ms — echo ISR bu süredir gelmediyse → E_STOP

// --- ST7735S TFT SPI Pin Tanımları ---
#define TFT_CS_PIN 10
#define TFT_DC_PIN 9
#define TFT_RST_PIN 8

// --- 4x4 Keypad Pin Tanımları ---
#define KP_SATIR_SAYISI 4
#define KP_SUTUN_SAYISI 4

// =============================================================================
// --- HMI (EKRAN VE TUŞ TAKIMI) NESNELERİ ---
// =============================================================================

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

const char tusHaritasi[KP_SATIR_SAYISI][KP_SUTUN_SAYISI] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
// Satır: A0, A1, A2, A3 | Sütun: A4, A5, D4, D7
byte satirPinleri[KP_SATIR_SAYISI] = {A0, A1, A2, A3};
byte sutunPinleri[KP_SUTUN_SAYISI] = {A4, A5, 4, 7};

Keypad keypad = Keypad(makeKeymap(tusHaritasi), satirPinleri, sutunPinleri,
                       KP_SATIR_SAYISI, KP_SUTUN_SAYISI);

// --- Renk Paleti (RGB565) ---
#define RENK_ARKAPLAN 0x0000 // Siyah
#define RENK_YAZI 0xFFFF     // Beyaz
#define RENK_BASLIK 0x07FF   // Cyan
#define RENK_ETIKET 0xFFE0   // Sarı
#define RENK_DEGER 0x07E0    // Yeşil
#define RENK_DOLUM 0x001F    // Mavi
#define RENK_BOSALTIM 0xF800 // Kırmızı

// =============================================================================
// --- SİSTEM DURUMLARI VE GLOBAL DEĞİŞKENLER ---
// =============================================================================

enum SistemDurumu {
  INIT_CALIBRATION, // Açılışta sensor kalibrasyonu
  IDLE,             // Bekleme
  SET_REFERENCE,    // Hedef su seviyesi girisi
  AUTO_CONTROL,     // PID döngüsü aktif
  TUNE_PID,         // Kp/Ki/Kd katsayılarını keypad'den ayarlama
  E_STOP            // Acil durdurma
};

volatile SistemDurumu mevcutDurum = INIT_CALIBRATION;

uint16_t referans_mm = 0;
uint16_t bosKapMesafe_mm = 0;
uint16_t maxSeviye_mm = 0;
int16_t aktif_pwm = 0; // Ekrana doğru PWM'i basmak için eklendi

// Dinamik bellek tahsisinden (String) kaçınmak için statik buffer
// Boyut 7: "10C50" gibi 5 karakter + ondalik + null = 7 byte
char girisBuffer[7] = {0};
uint8_t girisIndeksi = 0;

// --- Delta-Update Ekran Değişkenleri ---
SistemDurumu eski_durum = (SistemDurumu)0xFF;
uint16_t eski_referans_mm = 0xFFFF;
uint16_t eski_gercekSeviye_mm = 0xFFFF;
int16_t eski_pwm = 32767;

// --- TUNE_PID: Katsayı Ayar Modu Durumu ---
// tuneSeciliParam : 0=Kp, 1=Ki, 2=Kd
// tuneParamSecildi: false = seçim fazı, true = değer girişi fazı
uint8_t tuneSeciliParam = 0;
bool tuneParamSecildi = false;

// =============================================================================
// --- FAZ 1: KESME TABANLI SENSÖR OKUMA (ISR) ---
// =============================================================================
volatile unsigned long yankibaslangicZamani = 0;
volatile unsigned long yankiSuresi = 0;
// trigger_zamani_geldi: Timer1 ISR'ı sadece bu bayrağı set eder (ISR kısa
// tutulur). loop() bayrağı görünce DPM ile 10µs darbe gönderir → Blind-spot
// engellendi.
volatile bool trigger_zamani_geldi = false;
// pid_zamaniGeldi: loop() içinde trigger gönderildikten sonra set edilir.
volatile bool pid_zamaniGeldi = false;

// Sensor Heartbeat: Her geçerli echo darbesi bu zaman damgasını günceller.
// loop() içinde millis() - sonEchoZamani > SENSOR_TIMEOUT_MS → kablo koptu!
volatile unsigned long sonEchoZamani = 0;

void echo_ISR() {
  if (PIND & (1 << PD2)) { // Yükselen Kenar: Darbe başlıyor
    yankibaslangicZamani = micros();
  } else { // Düşen Kenar: Darbe bitti, süreyi hesapla
    yankiSuresi = micros() - yankibaslangicZamani;
    sonEchoZamani = millis(); // Heartbeat: sensör hayatta
  }
}

ISR(TIMER1_COMPA_vect) {
  // ISR YALNIZCA bayrak set eder — delayMicroseconds veya digitalWrite YASAK.
  // DPM trigger ve pid_zamaniGeldi → loop() içinde işlenir.
  if (mevcutDurum == AUTO_CONTROL || mevcutDurum == INIT_CALIBRATION) {
    trigger_zamani_geldi = true;
  }
}

// =============================================================================
// --- FAZ 2: DİJİTAL SİNYAL İŞLEME (DSP) KATMANI ---
// =============================================================================
uint16_t sonOlcumler[5] = {0, 0, 0, 0, 0};
uint8_t olcumIndeksi = 0;
uint16_t filtrelenmisMesafe_mm = 0;

void veriyiIsle(unsigned long yanki) {
  uint16_t anlikMesafe_mm = (uint16_t)((yanki * 10UL) / 58UL); // Float Yasak

  sonOlcumler[olcumIndeksi] = anlikMesafe_mm;
  olcumIndeksi = (olcumIndeksi + 1) % 5;

  uint16_t kopya[5];
  for (uint8_t i = 0; i < 5; i++)
    kopya[i] = sonOlcumler[i];

  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < (4 - i); j++) {
      if (kopya[j] > kopya[j + 1]) {
        uint16_t tmp = kopya[j];
        kopya[j] = kopya[j + 1];
        kopya[j + 1] = tmp;
      }
    }
  }
  uint16_t medyan = kopya[2];
  filtrelenmisMesafe_mm = (filtrelenmisMesafe_mm * 3 + medyan) / 4;
}

// =============================================================================
// --- FAZ 3: EYLEYİCİ SÜRÜŞÜ VE GÜVENLİK KİLİTLERİ (HAL) ---
// =============================================================================
// Shadow Register: Gerçek PWM değeri sadece DEĞİŞTİĞİNDE donanıma yazılır.
// Lüzumsuz analogWrite() çağrıları Timer0 interrupt yükünü ortadan kaldırır.
static uint8_t shadow_dolum = 0;
static uint8_t shadow_bosaltim = 0;

void pompaSur(int16_t kontrolSinyali) {
  uint8_t yeniDolum = 0;
  uint8_t yeniBosaltim = 0;

  if (kontrolSinyali > -OLUBANT_ESIGI && kontrolSinyali < OLUBANT_ESIGI) {
    // Ölü bant: her iki pompa kapalı
  } else {
    int16_t mutlak = abs(kontrolSinyali);
    uint8_t pwmDeger = (mutlak > PWM_MAX) ? (uint8_t)PWM_MAX : (uint8_t)mutlak;
    if (kontrolSinyali > 0) {
      yeniDolum = pwmDeger;
    } else {
      yeniBosaltim = pwmDeger;
    }
  }

  // Sadece değişen kanalı donanıma yaz (Shadow Register Pattern)
  if (yeniDolum != shadow_dolum) {
    analogWrite(DOLUM_POMPASI_PIN, yeniDolum);
    shadow_dolum = yeniDolum;
  }
  if (yeniBosaltim != shadow_bosaltim) {
    analogWrite(BOSALTIM_POMPASI_PIN, yeniBosaltim);
    shadow_bosaltim = yeniBosaltim;
  }
}

// =============================================================================
// --- FAZ 4: AYRIK ZAMANLI PID — Q8.8 FIXED-POINT IMPLEMENTASYONU ---
// =============================================================================
//
//  NEDEN Q8.8?
//  ATmega328P'de float = yazılımsal emülasyon = ~14 clock/işlem.
//  Tam sayı kaydırmaları = 1 clock. Bu mimari FPU maliyetini sıfırlar.
//
//  FORMAT: Gerçek değer = Q8_değeri / 256  (256 = 2^8)
//    Çarpma : a_q8 * b_int → Q8 sonuç (bir >> 8 ile gerçeğe döner)
//    Toplama: Q8 + Q8      → Q8 sonuç (doğrudan)
//
// --- Katsayı Tablosu (Gerçek → Q8) ---
//   Kp = 2.0  →  2.0 * 256 = 512
//   Ki = 0.5  →  0.5 * 256 = 128
//   Kd = 1.0  →  1.0 * 256 = 256
int16_t Kp_q8 = 512;
int16_t Ki_q8 = 128;
int16_t Kd_q8 = 256;

// İntegral biriktirici: Q8 formatında tutulur → int32_t şart (geniş alan)
// Anti-windup sınırı: ±255 gerçek birim = ±(255 × 256) = ±65280
int32_t integralToplam_q8 = 0;
const int32_t INTEGRAL_LIMIT_Q8 = (int32_t)255 * 256; // = 65280

// dt = 0.1 s (Timer1 CTC 10 Hz)
//   İntegral için : dt_q8  = 0.1 × 256 = 25.6 → 26 (yuvarlama, %0.15 hata)
//   Türev için    : INV_DT = 1 / 0.1 = 10 (TAM SAYI, kayıpsız)
const int16_t dt_q8 = 26;
const uint8_t INV_DT = 10;

uint16_t sonGercekSeviye = 0; // y[k-1]: önceki ölçüm (mm, dipten)

int16_t pidHesapla(uint16_t ref_mm, uint16_t gercek_mm) {

  // 1. HATA (e) — Q0 (normal tam sayı, mm)
  int16_t e = (int16_t)ref_mm - (int16_t)gercek_mm;

  // 2. ORANSAL (P) — Q8 formatında
  //    P_q8 = Kp_q8 × e
  //    P    = P_q8 >> 8  ≡  Kp × e
  //    Overflow: 512 × 1000 = 512000  < INT32_MAX ✓
  int32_t P_q8 = (int32_t)Kp_q8 * e;

  // 3. İNTEGRAL (I) — Q8 biriktirilir, Anti-Windup uygulanır
  //
  //    dI       = Ki × e × dt
  //    dI_q8    = (Ki_q8 × e × dt_q8) >> 8
  //
  //    Örnek doğrulama: Ki=0.5, e=100mm, dt=0.1
  //      Beklenen : 0.5 × 100 × 0.1 = 5.0
  //      Hesaplama: (128 × 100 × 26) >> 8 = 332800 >> 8 = 1300 (Q8)
  //                 1300 / 256 ≈ 5.08  (%1.5 hata, dt_q8 yuvarlama kaynaklı)
  //    Overflow: 128 × 1000 × 26 = 3328000 < INT32_MAX ✓
  integralToplam_q8 += ((int32_t)Ki_q8 * e * dt_q8) >> 8;

  if (integralToplam_q8 > INTEGRAL_LIMIT_Q8)
    integralToplam_q8 = INTEGRAL_LIMIT_Q8;
  else if (integralToplam_q8 < -INTEGRAL_LIMIT_Q8)
    integralToplam_q8 = -INTEGRAL_LIMIT_Q8;

  // 4. TÜREV (D) — Derivative on Measurement, referans şokunu engeller
  //
  //    D        = -Kd × (y[k] - y[k-1]) / dt
  //             = -Kd × delta_y × (1/dt)
  //             = -Kd × delta_y × INV_DT
  //
  //    NEDEN BÖLME YOK?
  //    dt = 0.1 sabitin tam tersi 1/dt = 10 tam sayıdır.
  //    Bölme yerine ×10 ile KAYIPSIZ hesaplanır (bölme hatası sıfır).
  //
  //    D_q8     = -(Kd_q8 × delta_y × INV_DT)
  //    D        = D_q8 >> 8  ≡  -Kd × delta_y / dt
  //
  //    Örnek doğrulama: Kd=1.0, delta_y=5mm, dt=0.1
  //      Beklenen : -(1.0 × 5 / 0.1) = -50
  //      Hesaplama: -(256 × 5 × 10)  = -12800 (Q8)
  //                 -12800 >> 8      = -50 ✓
  //    Overflow: 256 × 1000 × 10 = 2560000 < INT32_MAX ✓
  int16_t delta_y = (int16_t)gercek_mm - (int16_t)sonGercekSeviye;
  int32_t D_q8 = -(int32_t)Kd_q8 * delta_y * INV_DT;

  // 5. TOPLAM → Q8'den Q0'a dönüşüm (>> 8)
  //    Tüm terimler Q8 formatında → toplanabilir → tek kaydırmayla tamsayıya
  int32_t u_q8 = P_q8 + integralToplam_q8 + D_q8;
  int16_t u = (int16_t)(u_q8 >> 8);

  // 6. Durum güncelle: y[k] → y[k-1] (bir sonraki türev için)
  sonGercekSeviye = gercek_mm;

  return u;
}

// =============================================================================
// --- parseQ8: Float-Free Ondalik Dizgeden Q8.8 Dönüştürücü ---
// =============================================================================
// Giriş: "2C5" gibi bir dizi ('C' = ondalik nokta)
// Çıkış: Q8.8 formatında int16_t
//
// Algoritma (float yasak):
//   Q8 = (tamKisim × 256) + (ondalikKisim × 256) / ondalikCarpan
//
// Dogrulama:
//   "2C5"   → (2×256) + (5×256)/10    = 512 + 128 = 640   = 2.5   × 256 ✓
//   "10C50" → (10×256) + (50×256)/100  = 2560 + 128 = 2688 = 10.5  × 256 ✓
//   "2"     → (2×256) + 0              = 512             = 2.0   × 256 ✓
//   "C5"    → 0       + (5×256)/10    = 128             = 0.5   × 256 ✓
//
// Kesinşlik: Ondalik kisim tam bölünmediğinde tamsayı truncation olur.
//   "1C3"  → 256 + (3×256)/10 = 256 + 76 = 332   (gercek 332.8, hata %0.24)
// Bu hata PID kalibrasyonu için kabul edilebilir.
int16_t parseQ8(const char *buf) {
  int32_t tamKisim = 0;
  int32_t ondalikKisim = 0;
  int32_t ondalikCarpan = 1;
  bool ondalikMod = false;

  for (uint8_t i = 0; buf[i] != '\0'; i++) {
    char c = buf[i];
    if (c == 'C') {
      // 'C' = ondalik nokta; ikinci 'C' görmezden gelinir (güvenlik)
      ondalikMod = true;
    } else if (c >= '0' && c <= '9') {
      if (!ondalikMod) {
        // Tam kisim: basamak ekle
        tamKisim = tamKisim * 10 + (c - '0');
      } else {
        // Ondalik kisim: basamak ekle ve carpanı on katla
        ondalikKisim = ondalikKisim * 10 + (c - '0');
        ondalikCarpan = ondalikCarpan * 10;
      }
    }
  }

  // Q8 = (tamKisim × 256) + (ondalikKisim × 256) / ondalikCarpan
  int32_t sonuc = (tamKisim * 256L) + ((ondalikKisim * 256L) / ondalikCarpan);

  // OVERFLOW KORUMASI: int16_t max = 32767 → 127.99 gerçek değer.
  // Kullanıcı 128+ girerse Q8 negatife taşar → PID ters çalışır = felaket.
  // Kelepce: 32512 = 127.0 × 256 (üst sınır, güvenli)
  if (sonuc > 32512)
    sonuc = 32512;
  if (sonuc < 0)
    sonuc = 0; // Negatif katsayı anlamsız

  return (int16_t)sonuc;
}

// =============================================================================
// --- FAZ 6: TİTREŞİMSİZ EKRAN GÜNCELLEMESİ (DELTA YÖNTEMİ) ---
// =============================================================================
void ekranGuncelle(SistemDurumu durum, uint16_t ref, uint16_t gercek,
                   int16_t pwm) {
  if (durum != eski_durum) {
    tft.fillScreen(RENK_ARKAPLAN);
    switch (durum) {
    case INIT_CALIBRATION:
      // ref parametresi bu durumda alt-faz göstergesidur:
      //   ref == 0  → Bekleme: kullanıcıdan onay bekleniyor
      //   ref == 1  → Ölçüm: kal. döngüsü çalışıyor
      // Her iki alt-fazın statik ekranları ayrı çizilir.
      // (Delta-update burada ref farkıyla tetiklenir)
      if (ref == 0) {
        // --- Aşama 1: Kullanıcı uyarısı ---
        tft.setTextColor(RENK_BOSALTIM, RENK_ARKAPLAN); // Kırmızı – dikkat!
        tft.setTextSize(1);
        tft.setCursor(8, 8);
        tft.print(F("GUVENLi KALiBRASYON"));
        tft.drawFastHLine(0, 20, tft.width(), RENK_BOSALTIM);
        tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
        tft.setCursor(8, 30);
        tft.print(F("KABI TAMAMEN BOSALT!"));
        tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
        tft.setCursor(8, 50);
        tft.print(F("Kap bos ise onlayin:"));
        tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
        tft.setTextSize(2);
        tft.setCursor(8, 70);
        tft.print(F("[#] BASLAT"));
        tft.setTextSize(1);
      } else {
        // --- Aşama 2: Ölçüm ekranı ---
        tft.setTextColor(RENK_BASLIK, RENK_ARKAPLAN);
        tft.setTextSize(1);
        tft.setCursor(8, 8);
        tft.print(F("KALiBRASYON"));
        tft.drawFastHLine(0, 20, tft.width(), RENK_BASLIK);
        tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
        tft.setCursor(8, 30);
        tft.print(F("Olculuyor..."));
        tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
        tft.setCursor(8, 50);
        tft.print(F("Lutfen bekleyin."));
      }
      break;
    case IDLE:
      tft.setTextColor(RENK_BASLIK, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(8, 8);
      tft.print(F("SU SEVIYE KONTROL"));
      tft.drawFastHLine(0, 20, tft.width(), RENK_BASLIK);
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 35);
      tft.print(F("[*] Seviye Ayarla"));
      tft.setCursor(8, 50);
      tft.print(F("[B] PID Ayarla"));
      break;
    case SET_REFERENCE:
      tft.setTextColor(RENK_BASLIK, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(8, 8);
      tft.print(F("HEDEF SEVIYE GIR"));
      tft.drawFastHLine(0, 20, tft.width(), RENK_BASLIK);
      tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
      tft.setCursor(8, 30);
      tft.print(F("Deger (mm):"));
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 85);
      tft.print(F("[#] Onayla  [*] Iptal"));
      break;
    case AUTO_CONTROL:
      tft.setTextColor(RENK_BASLIK, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(8, 8);
      tft.print(F("OTO KONTROL"));
      tft.drawFastHLine(0, 20, tft.width(), RENK_BASLIK);
      tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
      tft.setCursor(8, 30);
      tft.print(F("Hedef :"));
      tft.setCursor(8, 46);
      tft.print(F("Gercek:"));
      tft.setCursor(8, 62);
      tft.print(F("Pompa :"));
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 100);
      tft.print(F("[*] Dur"));
      break;
    case TUNE_PID:
      // -------------------------------------------------------------------
      // TUNE_PID statik çerçeve:
      //   Başlık + mevcut Kp/Ki/Kd değerleri (ondalık) + talimatlar
      //   Katsayı değiştikten sonra eski_durum=0xFF ile yeniden çizilir.
      //
      // Q8 → ondalık gösterim (float yok):
      //   Kp_q8=640 → 640>>8=2, ((128*10)>>8)=5 → ekranda "2.5"
      //   Ki_q8=128 → 128>>8=0, ((128*10)>>8)=5 → ekranda "0.5"
      // -------------------------------------------------------------------
      tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN); // Sarı = ayar modu
      tft.setTextSize(1);
      tft.setCursor(8, 8);
      tft.print(F("PID AYAR MODU"));
      tft.drawFastHLine(0, 20, tft.width(), RENK_ETIKET);
      tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
      tft.setCursor(8, 28);
      tft.print(F("Kp:"));
      tft.setCursor(8, 40);
      tft.print(F("Ki:"));
      tft.setCursor(8, 52);
      tft.print(F("Kd:"));
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      // Q8 → ondalık gösterim (float yok): tamKisim.ondalikKisim
      tft.setCursor(80, 28);
      tft.print(Kp_q8 >> 8);
      tft.print('.');
      tft.print(((Kp_q8 & 0xFF) * 10) >> 8);
      tft.setCursor(80, 40);
      tft.print(Ki_q8 >> 8);
      tft.print('.');
      tft.print(((Ki_q8 & 0xFF) * 10) >> 8);
      tft.setCursor(80, 52);
      tft.print(Kd_q8 >> 8);
      tft.print('.');
      tft.print(((Kd_q8 & 0xFF) * 10) >> 8);
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 66);
      tft.print(F("[1]Kp [2]Ki [3]Kd"));
      tft.setCursor(8, 78);
      tft.print(F("[#]Kaydet [A]Sil [*]Cik"));
      break;
    case E_STOP:
      tft.setTextColor(RENK_BOSALTIM, RENK_ARKAPLAN);
      tft.setTextSize(2);
      tft.setCursor(8, 15);
      tft.print(F("!!! HATA !!!"));
      tft.setTextSize(1);
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 45);
      tft.print(F("Motorlar durduruldu."));
      tft.setCursor(8, 60);
      tft.print(F("RESET gereklidir."));
      break;
    }
    eski_durum = durum;
    eski_referans_mm = 0xFFFF;
    eski_gercekSeviye_mm = 0xFFFF;
    eski_pwm = 32767;
  }

  if (durum == SET_REFERENCE) {
    if (ref != eski_referans_mm) {
      tft.fillRect(8, 44, 120, 14, RENK_ARKAPLAN);
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(8, 46);
      if (girisIndeksi == 0)
        tft.print(F("_"));
      else {
        tft.print(girisBuffer);
        tft.print(F(" mm"));
      }
      eski_referans_mm = ref;
    }
  } else if (durum == AUTO_CONTROL) {
    if (ref != eski_referans_mm) {
      tft.fillRect(60, 30, 68, 12, RENK_ARKAPLAN);
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(60, 30);
      tft.print(ref);
      tft.print(F(" mm"));
      eski_referans_mm = ref;
    }
    if (gercek != eski_gercekSeviye_mm) {
      tft.fillRect(60, 46, 68, 12, RENK_ARKAPLAN);
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(60, 46);
      tft.print(gercek);
      tft.print(F(" mm"));
      eski_gercekSeviye_mm = gercek;
    }
    if (pwm != eski_pwm) {
      tft.fillRect(60, 62, 68, 12, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(60, 62);
      if (pwm > 0) {
        tft.setTextColor(RENK_DOLUM, RENK_ARKAPLAN);
        tft.print('+');
        tft.print(pwm);
        tft.print(F(" D"));
      } else if (pwm < 0) {
        tft.setTextColor(RENK_BOSALTIM, RENK_ARKAPLAN);
        tft.print(pwm);
        tft.print(F(" B"));
      } else {
        tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
        tft.print(F("OLUBANT"));
      }
      eski_pwm = pwm;
    }
  } else if (durum == TUNE_PID) {
    // Delta-update: seçilen parametre, faz ve giriş uzunluğu değişince
    // güncelle. Tek bir int16_t'ye encode edilen bileşik durum (eski_pwm bu
    // durumda bunu taşır):
    //   bit 15-8 : tuneSeciliParam
    //   bit 7    : tuneParamSecildi
    //   bit 6-0  : girisIndeksi
    int16_t tuneState =
        (int16_t)(((uint16_t)tuneSeciliParam << 8) |
                  (tuneParamSecildi ? 0x80 : 0x00) | (girisIndeksi & 0x7F));
    if (tuneState != eski_pwm) {
      tft.fillRect(8, 90, 152, 26, RENK_ARKAPLAN); // Input alanını temizle
      tft.setTextSize(1);
      if (tuneParamSecildi) {
        // Hangi katsayı seçildiyse adını göster
        const char *isimler[3] = {"Kp", "Ki", "Kd"};
        tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
        tft.setCursor(8, 92);
        tft.print(isimler[tuneSeciliParam]);
        tft.print(F(" yeni deger:"));
        tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
        tft.setCursor(8, 106);
        if (girisIndeksi == 0) {
          tft.print(F("_"));
        } else {
          // 'C' karakterini ekranda '.' olarak göster (görsel manipulasyon)
          for (uint8_t i = 0; i < girisIndeksi; i++) {
            tft.print(girisBuffer[i] == 'C' ? '.' : girisBuffer[i]);
          }
        }
      } else {
        tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
        tft.setCursor(8, 95);
        tft.print(F("Hangi? [1]Kp [2]Ki [3]Kd"));
      }
      eski_pwm = tuneState;
    }
  }
}

// =============================================================================
// --- KURULUM ---
// =============================================================================
void setup() {
  Serial.begin(115200); // Telemetri darboğazı açıldı: 9600 → 115200 baud

  // TRIGGER pini: DPM — pinMode'den ~8x hızlı, ISR uyumlu
  DDRD |= (1 << DDD3);  // D3 → çıkış
  PORTD &= ~(1 << PD3); // D3 → LOW (başlangıç)

  pinMode(ECHO_PIN, INPUT);
  pinMode(DOLUM_POMPASI_PIN, OUTPUT);
  pinMode(BOSALTIM_POMPASI_PIN, OUTPUT);

  analogWrite(DOLUM_POMPASI_PIN, 0);
  analogWrite(BOSALTIM_POMPASI_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echo_ISR, CHANGE);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1561;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(RENK_ARKAPLAN);
  tft.setTextWrap(false);

  keypad.setDebounceTime(10);
  ekranGuncelle(INIT_CALIBRATION, 0, 0, 0);

  wdt_enable(WDTO_500MS); // Watchdog aktif: 500ms içinde wdt_reset() gelmezse →
                          // donanımsal reset
}

// =============================================================================
// --- ANA DÖNGÜ (STATE MACHINE) ---
// =============================================================================
void loop() {
  wdt_reset(); // Watchdog beslendi — sistem hayatta, loop donmuyor

  // Asenkron DPM Tetikleyici (Blind-spot Çözümü)
  // ISR sadece bayrak set eder; darbe DPM ile loop() içinde gönderilir.
  // Bu sayede ISR içinde yasak olan delayMicroseconds() ortadan kalktı.
  if (trigger_zamani_geldi) {
    trigger_zamani_geldi = false;
    PORTD |= (1 << PD3);   // HIGH — DPM: 2 clock cycle
    delayMicroseconds(10); // HC-SR04 zorunlu 10µs darbe genişliği
    PORTD &= ~(1 << PD3);  // LOW
    if (mevcutDurum == AUTO_CONTROL) {
      pid_zamaniGeldi = true; // PID döngüsünü tetikle
    }
  }

  char tus = keypad.getKey();
  if (tus == NO_KEY)
    tus = 0;

  // Her Loop Dönüşünde Gerçek Seviye Koordinat Dönüşümü
  uint16_t anlik_su_yuksekligi = 0;
  if (filtrelenmisMesafe_mm < bosKapMesafe_mm) {
    anlik_su_yuksekligi = bosKapMesafe_mm - filtrelenmisMesafe_mm;
  }

  switch (mevcutDurum) {

  case INIT_CALIBRATION: {
    // =========================================================================
    // GÜVENLİ KALİBRASYON (Power-Loss Anömali Koruması)
    // =========================================================================
    // Sorun: Güç kesilip geldiğinde, kap dolu olabilir.
    //        Otonom kal. bu durumda suyun yüzeyini "taban" sanacak → felaket.
    // Çözüm: Kalibrasyon için insan onayı zorunlu kılındı (“Secure Handshake”).
    //
    // static bool: Bu bayrak YALNIZCA ilk reset'te başlatılır.
    // Durum ieinde loop'u n kez döndürmesi gerektiğinden global olması şart;
    // static anahtar sözcüğü onu stack yerine BSS bölgüsine (global alan)
    // taşır, 2KB SRAM'dan sadece 1 bit tüketir.
    static bool onayBekleniyor = true;

    if (onayBekleniyor) {
      // --- Aşama 1: Kullanıcı 'kap boş' onayı bekle ---
      // Motorlar kesinlikle durur; # gelene kadar sonsuz döngüde kalır.
      pompaSur(0);

      // ekranGuncelle: ref=0 → uyarı ekranı
      ekranGuncelle(INIT_CALIBRATION, 0, 0, 0);

      if (tus == '#') {
        // Kullanıcı kabın boş olduğunu onayladı.
        onayBekleniyor = false;
        // ekranGuncelle'yi zorla güncellemeye itmek için eski_durum’u sıfırla.
        // (Durum değişmedi fakat alt-faz değişti → farklı statik ekran.)
        eski_durum = (SistemDurumu)0xFF;
        Serial.println(F("[KALIBRASYON] Onay alindi. Olcum basliyor..."));
      }
      // Onay gelmedi → bu loop turunu bitir, bir sonrakini bekle.
      break;
    }

    // --- Aşama 2: Kullanıcı onayladı → Kal. ölçüm döngüsü ---
    // ekranGuncelle: ref=1 → “Ölçülüyor...” ekranı
    ekranGuncelle(INIT_CALIBRATION, 1, 0, 0);

    uint32_t toplam = 0;
    const uint8_t N = 10;
    for (uint8_t i = 0; i < N; i++) {
      wdt_reset(); // WDT KORUMASI: 10×60ms = 600ms > 500ms WDT → bu olmadan
                   // BOOTLOOP!

      // DPM Trigger: digitalWrite'den ~8x hızlı
      PORTD |= (1 << PD3);   // HIGH (2 clock cycle)
      delayMicroseconds(10); // HC-SR04 zorunlu 10µs
      PORTD &= ~(1 << PD3);  // LOW
      delay(60);             // HC-SR04 min. döngü süresi

      noInterrupts();
      unsigned long yanki = yankiSuresi;
      interrupts();

      // Henüz geçerli echo gelmemişse bu ölçümü atla (güvenli başlangıç)
      // KORUMA: Sensör hiç yanıt vermezse sonsuz döngü riski vardı.
      // wdt_reset() WDT'yi beslediği için watchdog tetiklenmezdi → sistem
      // donar. Retry limiti: 20 deneme × 10ms = 200ms sonra E_STOP.
      if (yanki == 0) {
        static uint8_t echoRetry = 0;
        if (++echoRetry > 20) {
          Serial.println(F("[E_STOP] Kalibrasyon: sensor yanit vermiyor!"));
          mevcutDurum = E_STOP;
          echoRetry = 0;
          break;
        }
        i--;
        wdt_reset();
        delay(10);
        continue;
      }

      veriyiIsle(yanki);
      toplam += filtrelenmisMesafe_mm;
    }

    bosKapMesafe_mm = (uint16_t)(toplam / N);
    const uint16_t SENSOR_KOR_NOKTASI_MM = 20;
    maxSeviye_mm = (bosKapMesafe_mm > SENSOR_KOR_NOKTASI_MM)
                       ? (bosKapMesafe_mm - SENSOR_KOR_NOKTASI_MM)
                       : 0;

    Serial.print(F("[KALIBRASYON] Bos kap mesafesi : "));
    Serial.print(bosKapMesafe_mm);
    Serial.println(F(" mm"));
    Serial.print(F("[KALIBRASYON] Maks su yuksekligi: "));
    Serial.print(maxSeviye_mm);
    Serial.println(F(" mm"));

    // Bayrağı sıfırla: bir sonraki reset'te kal. tekrar başlayacak.
    onayBekleniyor = true;
    mevcutDurum = IDLE;
    break;
  }

  case IDLE: {
    pompaSur(0);
    if (tus == '*') {
      memset(girisBuffer, 0, sizeof(girisBuffer));
      girisIndeksi = 0;
      mevcutDurum = SET_REFERENCE;
    } else if (tus == 'B') {
      // PID Ayar Moduna geçiş: buffer + durum sıfırla
      memset(girisBuffer, 0, sizeof(girisBuffer));
      girisIndeksi = 0;
      tuneSeciliParam = 0;
      tuneParamSecildi = false;
      mevcutDurum = TUNE_PID;
    }
    break;
  }

  case SET_REFERENCE: {
    if (tus >= '0' && tus <= '9') {
      if (girisIndeksi < 4) {
        girisBuffer[girisIndeksi] = tus;
        girisIndeksi++;
        girisBuffer[girisIndeksi] = '\0';
      }
    }
    // YAMA 1: Geri Silme (Backspace)
    // 'A' tuşu son karakteri siler. İndeks 0'daysa (buffer boş) işlem yapılmaz.
    else if (tus == 'A' && girisIndeksi > 0) {
      girisIndeksi--;
      girisBuffer[girisIndeksi] = '\0';
    } else if (tus == '#') {
      uint16_t girilen = (uint16_t)atoi(girisBuffer);
      if (girilen > 0 && girilen <= maxSeviye_mm) {
        referans_mm = girilen;
        integralToplam_q8 = 0; // Q8 format sıfırla (windup temizle)
        // sonGercekSeviye: PID türev için y[k-1] başlatılıyor.
        // Mevcut su yüksekliğine eşitlenir → ilk turda türev şoku sıfır.
        sonGercekSeviye = anlik_su_yuksekligi;
        Serial.print(F("[HEDEF] "));
        Serial.print(referans_mm);
        Serial.println(F(" mm"));
        mevcutDurum = AUTO_CONTROL;
      } else {
        Serial.println(F("[HATA] Gecersiz deger, tekrar girin."));
        memset(girisBuffer, 0, sizeof(girisBuffer));
        girisIndeksi = 0;
      }
    } else if (tus == '*') {
      memset(girisBuffer, 0, sizeof(girisBuffer));
      girisIndeksi = 0;
      mevcutDurum = IDLE;
    }
    // Ekranı SADECE hâlâ SET_REFERENCE durumundaysak güncelle.
    // Geçiş olduysa döngü sonu ekranGuncelle halleder.
    // girisIndeksi ref parametresi olarak geçilir → delta-update tetiklenir.
    if (mevcutDurum == SET_REFERENCE) {
      ekranGuncelle(SET_REFERENCE, girisIndeksi, 0, 0);
    }
    break;
  }

  case AUTO_CONTROL: {
    if (pid_zamaniGeldi) {
      pid_zamaniGeldi = false;

      // Atomik veri kopyası
      noInterrupts();
      unsigned long yanki = yankiSuresi;
      interrupts();

      veriyiIsle(yanki); // filtrelenmisMesafe_mm güncellendi

      // FIX: Taze veri ile koordinat dönüşümü (veriyiIsle SONRASI)
      // anlik_su_yuksekligi burada güncellenir → E_STOP ve PID taze veri
      // kullanır, döngü sonu ekranGuncelle de aynı taze veriyi görür.
      anlik_su_yuksekligi = (filtrelenmisMesafe_mm < bosKapMesafe_mm)
                                ? (bosKapMesafe_mm - filtrelenmisMesafe_mm)
                                : 0;

      // GÜVENLİK 1: Sensör Heartbeat Timeout
      // echo_ISR her geçerli düşen kenarda sonEchoZamani'ni günceller.
      // Bu zaman damgası SENSOR_TIMEOUT_MS'dir duruyorsa → kablo koptu/sensör
      // öldü. filtrelenmisMesafe_mm == 0 kontrolü YETERSİZ: IIR filtre eski
      // değeri tutar! NOT: sonEchoZamani 4 byte = non-atomic, atomik kopyalama
      // şart.
      noInterrupts();
      unsigned long echoKopya = sonEchoZamani;
      interrupts();
      if (millis() - echoKopya > SENSOR_TIMEOUT_MS) {
        Serial.println(F("[E_STOP] Sensor heartbeat timeout! Kablo koptu?"));
        mevcutDurum = E_STOP;
        break;
      }

      // GÜVENLİK 2: Su taşması
      if (anlik_su_yuksekligi > maxSeviye_mm) {
        Serial.println(F("[E_STOP] Tasma tespit edildi!"));
        mevcutDurum = E_STOP;
        break;
      }

      static uint8_t stallSayaci = 0;

      // y[k-1] değerini PID çağrısından ÖNCE kaydet (PID içinde üzerine yazar)
      uint16_t oncekiSeviye = sonGercekSeviye;
      int16_t u = pidHesapla(referans_mm, anlik_su_yuksekligi);

      // GÜVENLİK 3: Kuru Çalışma Koruması (Dry-Run Protection)
      if (anlik_su_yuksekligi <= 5 && u < 0) {
        Serial.println(F("[E_STOP] Kuru calisma tespiti!"));
        mevcutDurum = E_STOP;
        break;
      }

      // GÜVENLİK 4: Tıkanıklık Tespiti (Stall Detection + Jitter Tolerance)
      // ESKİ: anlik == onceki → sensör jitterı (1-2mm) sürekli sıfırlar
      // YENİ: abs(anlik - onceki) < STALL_TOLERANCE → dalga titremesini emer
      int16_t seviyeDelta =
          (int16_t)anlik_su_yuksekligi - (int16_t)oncekiSeviye;
      if (abs(u) > 200 && abs(seviyeDelta) < STALL_TOLERANCE) {
        stallSayaci++;
        if (stallSayaci >= 50) { // 50 × 100ms = 5 saniye
          Serial.println(F("[E_STOP] Stall tespiti!"));
          mevcutDurum = E_STOP;
          break;
        }
      } else {
        stallSayaci = 0;
      }

      aktif_pwm = u;
      pompaSur(u);

      Serial.print(F("H:"));
      Serial.print(referans_mm);
      Serial.print(F(" G:"));
      Serial.print(anlik_su_yuksekligi);
      Serial.print(F(" U:"));
      Serial.print(u);
      Serial.print(F(" STALL:"));
      Serial.println(stallSayaci);
    }

    if (tus == '*') {
      pompaSur(0);
      aktif_pwm = 0;
      mevcutDurum = IDLE;
    }
    break;
  }

  case TUNE_PID: {
    // =========================================================================
    // PID KATSAYI AYAR MODU — parseQ8 ile Float-Free Ondalik Giriş
    // =========================================================================
    // Giriş formatı: Gerçek değer doğrudan, ondalik nokta için [C] tuşu.
    //   Örnek: Kp = 2.5 için [2][C][5][#] tuşlarına bas.
    //   Örnek: Ki = 0.5 için [C][5][#] veya [0][C][5][#]
    //   Ekranda 'C' karakteri '.' olarak görünür.
    //
    // Buffer limiti: 6 karakter ("10C50" + null = 7 byte)
    // Tek 'C' kuralı: buffer'da zaten 'C' varsa ikinci 'C' tuşu reddedilir.

    if (!tuneParamSecildi) {
      // --- Aşama 1: Katsayı seçimi ---
      if (tus == '1') {
        tuneSeciliParam = 0;
        tuneParamSecildi = true;
        memset(girisBuffer, 0, 7);
        girisIndeksi = 0;
      } else if (tus == '2') {
        tuneSeciliParam = 1;
        tuneParamSecildi = true;
        memset(girisBuffer, 0, 7);
        girisIndeksi = 0;
      } else if (tus == '3') {
        tuneSeciliParam = 2;
        tuneParamSecildi = true;
        memset(girisBuffer, 0, 7);
        girisIndeksi = 0;
      } else if (tus == '*') {
        mevcutDurum = IDLE;
      }
    } else {
      // --- Aşama 2: Değer girişi ---
      if (tus >= '0' && tus <= '9') {
        if (girisIndeksi < 6) {
          girisBuffer[girisIndeksi++] = tus;
          girisBuffer[girisIndeksi] = '\0';
        }
      } else if (tus == 'C') {
        // Ondalik nokta: buffer'da henüz 'C' yoksa ekle
        // Kontrol: strchr yerine elle tara (Arduino kütüphane bağımsızlığı)
        bool cVarMi = false;
        for (uint8_t i = 0; i < girisIndeksi; i++) {
          if (girisBuffer[i] == 'C') {
            cVarMi = true;
            break;
          }
        }
        if (!cVarMi && girisIndeksi < 6) {
          girisBuffer[girisIndeksi++] = 'C';
          girisBuffer[girisIndeksi] = '\0';
        }
      } else if (tus == 'A' && girisIndeksi > 0) {
        girisBuffer[--girisIndeksi] = '\0';
      } else if (tus == '#') {
        // parseQ8: 'C' ondalik noktası içeren dizgiyi Q8.8'e çevir
        int16_t yeniQ8 = parseQ8(girisBuffer);
        if (yeniQ8 > 0) { // Sıfır veya negatif geçersiz
          if (tuneSeciliParam == 0) {
            Kp_q8 = yeniQ8;
            Serial.print(F("[TUNE] Kp="));
          } else if (tuneSeciliParam == 1) {
            Ki_q8 = yeniQ8;
            integralToplam_q8 = 0;
            Serial.print(F("[TUNE] Ki="));
          } else {
            Kd_q8 = yeniQ8;
            Serial.print(F("[TUNE] Kd="));
          }
          // Serial çıktısı: Q8'den geri gerçeğe (tam ve ondalik) — float yok
          Serial.print(yeniQ8 >> 8); // Tam kisim
          Serial.print('.');
          Serial.println(((yeniQ8 & 0xFF) * 10) >> 8); // Ondalik kisim (1 hane)
          eski_durum = (SistemDurumu)0xFF;             // Ekrani zorla yenile
          tuneParamSecildi = false;
          memset(girisBuffer, 0, 7);
          girisIndeksi = 0;
        } else {
          Serial.println(F("[TUNE] Gecersiz deger."));
          memset(girisBuffer, 0, 7);
          girisIndeksi = 0;
        }
      } else if (tus == '*') {
        // Aşama 2'den Aşama 1'e dön
        tuneParamSecildi = false;
        memset(girisBuffer, 0, 7);
        girisIndeksi = 0;
        eski_durum = (SistemDurumu)0xFF;
      }
    }
    if (mevcutDurum == TUNE_PID) {
      ekranGuncelle(TUNE_PID, girisIndeksi, tuneSeciliParam,
                    (int16_t)tuneParamSecildi);
    }
    break;
  }

  case E_STOP: {
    analogWrite(DOLUM_POMPASI_PIN, 0);
    analogWrite(BOSALTIM_POMPASI_PIN, 0);
    shadow_dolum = 0; // Shadow register tutarlılığı
    shadow_bosaltim = 0;
    aktif_pwm = 0;
    break;
  }

  } // Switch Sonu

  // Sadece SET_REFERENCE durumunda değilsek normal ekran güncellemesini çağır
  if (mevcutDurum != SET_REFERENCE && mevcutDurum != TUNE_PID) {
    ekranGuncelle(mevcutDurum, referans_mm, anlik_su_yuksekligi, aktif_pwm);
  }
}

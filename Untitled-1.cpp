// =============================================================================
// Kapalı Çevrim Su Seviyesi Kontrol Sistemi (v1.0 - Endüstriyel Sürüm)
// Mimari: Olay-Güdümlü Durum Makinesi (Event-Driven State Machine)
// İşlemci: ATmega328P (16 MHz) / 2KB SRAM Optimizasyonlu
// =============================================================================

#include <Adafruit_GFX.h>    // Grafik temeli (Adafruit)
#include <Adafruit_ST7735.h> // ST7735S TFT sürücüsü
#include <Arduino.h>
#include <Keypad.h> // 4x4 matris tuş takımı
#include <SPI.h>    // ST7735 SPI veriyolu
#include <stdlib.h> // atoi() ve bellek fonksiyonları

// =============================================================================
// --- DONANIM SABİTLERİ (HAL Katmanı) ---
// =============================================================================
#define TRIGGER_PIN 3          // HC-SR04 Trigger → D3
#define ECHO_PIN 2             // HC-SR04 Echo    → D2 (INT0)
#define DOLUM_POMPASI_PIN 5    // Dolum Pompası   → D5 (Timer0 PWM)
#define BOSALTIM_POMPASI_PIN 6 // Boşaltım Pompası→ D6 (Timer0 PWM)

#define OLUBANT_ESIGI 40     // Bu PWM'in altında motorlar dönmez
#define PWM_MAX 255          // analogWrite maksimum sınırı
#define INTEGRAL_LIMIT 255.0 // Anti-Windup sınırı

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
char girisBuffer[5] = {0};
uint8_t girisIndeksi = 0;

// --- Delta-Update Ekran Değişkenleri ---
SistemDurumu eski_durum = (SistemDurumu)0xFF;
uint16_t eski_referans_mm = 0xFFFF;
uint16_t eski_gercekSeviye_mm = 0xFFFF;
int16_t eski_pwm = 32767;

// --- TUNE_PID: Katsayı Ayar Modu Durumu ---
// tuneSeciliParam : 0=Kp, 1=Ki, 2=Kd
// tuneParamSecildi: false = seçim fazı, true = değer girişi fazı
uint8_t tuneSeciliParam  = 0;
bool    tuneParamSecildi = false;

// =============================================================================
// --- FAZ 1: KESME TABANLI SENSÖR OKUMA (ISR) ---
// =============================================================================
volatile unsigned long yankibaslangicZamani = 0;
volatile unsigned long yankiSuresi = 0;
// NOT: pid_zamaniGeldi Timer1 ISR tarafından set edilir;
// loop() okur ve AUTO_CONTROL içinde sıfırlar.
volatile bool pid_zamaniGeldi = false;

void echo_ISR() {
  if (PIND & (1 << PD2)) { // Yükselen Kenar: Darbe başlıyor
    yankibaslangicZamani = micros();
  } else {                  // Düşen Kenar: Darbe bitti, süreyi hesapla
    yankiSuresi = micros() - yankibaslangicZamani;
  }
}

ISR(TIMER1_COMPA_vect) {
  if (mevcutDurum == AUTO_CONTROL) {
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    pid_zamaniGeldi = true;
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
void pompaSur(int16_t kontrolSinyali) {
  if (kontrolSinyali > -OLUBANT_ESIGI && kontrolSinyali < OLUBANT_ESIGI) {
    analogWrite(DOLUM_POMPASI_PIN, 0);
    analogWrite(BOSALTIM_POMPASI_PIN, 0);
    return;
  }

  int16_t mutlak = abs(kontrolSinyali);
  uint8_t pwmDeger = (mutlak > PWM_MAX) ? (uint8_t)PWM_MAX : (uint8_t)mutlak;

  if (kontrolSinyali > 0) {
    analogWrite(BOSALTIM_POMPASI_PIN, 0);
    analogWrite(DOLUM_POMPASI_PIN, pwmDeger);
  } else {
    analogWrite(DOLUM_POMPASI_PIN, 0);
    analogWrite(BOSALTIM_POMPASI_PIN, pwmDeger);
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
const int16_t dt_q8  = 26;
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

  if      (integralToplam_q8 >  INTEGRAL_LIMIT_Q8) integralToplam_q8 =  INTEGRAL_LIMIT_Q8;
  else if (integralToplam_q8 < -INTEGRAL_LIMIT_Q8) integralToplam_q8 = -INTEGRAL_LIMIT_Q8;

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
  int32_t D_q8    = -(int32_t)Kd_q8 * delta_y * INV_DT;

  // 5. TOPLAM → Q8'den Q0'a dönüşüm (>> 8)
  //    Tüm terimler Q8 formatında → toplanabilir → tek kaydırmayla tamsayıya
  int32_t u_q8 = P_q8 + integralToplam_q8 + D_q8;
  int16_t u    = (int16_t)(u_q8 >> 8);

  // 6. Durum güncelle: y[k] → y[k-1] (bir sonraki türev için)
  sonGercekSeviye = gercek_mm;

  return u;
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
      tft.setCursor(8, 35); tft.print(F("[*] Seviye Ayarla"));
      tft.setCursor(8, 50); tft.print(F("[B] PID Ayarla"));
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
      //   Başlık + mevcut Kp/Ki/Kd değerleri + talimatlar
      //   Katsayı değiştikten sonra eski_durum=0xFF ile yeniden çizilir.
      //
      // Q8 → x10 dönüşümü (float yok):
      //   Kp_q8=512 → (512*10)>>8 = 20 → gösterim "20" (= 2.0)
      //   Ki_q8=128 → (128*10)>>8 = 5  → gösterim "5"  (= 0.5)
      // -------------------------------------------------------------------
      tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN); // Sarı = ayar modu
      tft.setTextSize(1);
      tft.setCursor(8, 8);  tft.print(F("PID AYAR MODU"));
      tft.drawFastHLine(0, 20, tft.width(), RENK_ETIKET);
      tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
      tft.setCursor(8,  28); tft.print(F("Kp (x10):"));
      tft.setCursor(8,  40); tft.print(F("Ki (x10):"));
      tft.setCursor(8,  52); tft.print(F("Kd (x10):"));
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setCursor(80, 28); tft.print((uint16_t)(((int32_t)Kp_q8 * 10L) >> 8));
      tft.setCursor(80, 40); tft.print((uint16_t)(((int32_t)Ki_q8 * 10L) >> 8));
      tft.setCursor(80, 52); tft.print((uint16_t)(((int32_t)Kd_q8 * 10L) >> 8));
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 66); tft.print(F("[1]Kp [2]Ki [3]Kd"));
      tft.setCursor(8, 78); tft.print(F("[#]Kaydet [A]Sil [*]Cik"));
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
    // Delta-update: seçilen parametre, faz ve giriş uzunluğu değişince güncelle.
    // Tek bir int16_t'ye encode edilen bileşik durum (eski_pwm bu durumda bunu taşır):
    //   bit 15-8 : tuneSeciliParam
    //   bit 7    : tuneParamSecildi
    //   bit 6-0  : girisIndeksi
    int16_t tuneState = (int16_t)(((uint16_t)tuneSeciliParam << 8)
                                 | (tuneParamSecildi ? 0x80 : 0x00)
                                 | (girisIndeksi & 0x7F));
    if (tuneState != eski_pwm) {
      tft.fillRect(8, 90, 152, 26, RENK_ARKAPLAN); // Input alanını temizle
      tft.setTextSize(1);
      if (tuneParamSecildi) {
        // Hangi katsayı seçildiyse adını göster
        const char* isimler[3] = {"Kp", "Ki", "Kd"};
        tft.setTextColor(RENK_ETIKET, RENK_ARKAPLAN);
        tft.setCursor(8, 92);
        tft.print(isimler[tuneSeciliParam]);
        tft.print(F(" yeni deger (x10):"));
        tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
        tft.setCursor(8, 106);
        if (girisIndeksi == 0) tft.print(F("_"));
        else                   tft.print(girisBuffer);
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
  Serial.begin(9600);

  pinMode(TRIGGER_PIN, OUTPUT);
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
}

// =============================================================================
// --- ANA DÖNGÜ (STATE MACHINE) ---
// =============================================================================
void loop() {
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
      digitalWrite(TRIGGER_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_PIN, LOW);
      delay(60);

      noInterrupts();
      unsigned long yanki = yankiSuresi;
      interrupts();

      // Henüz geçerli echo gelmemişse bu ölçümü atla (güvenli başlangıç)
      if (yanki == 0) { i--; delay(10); continue; }

      veriyiIsle(yanki);
      toplam += filtrelenmisMesafe_mm;
    }

    bosKapMesafe_mm = (uint16_t)(toplam / N);
    const uint16_t SENSOR_KOR_NOKTASI_MM = 20;
    maxSeviye_mm = (bosKapMesafe_mm > SENSOR_KOR_NOKTASI_MM)
                       ? (bosKapMesafe_mm - SENSOR_KOR_NOKTASI_MM) : 0;

    Serial.print(F("[KALIBRASYON] Bos kap mesafesi : ")); Serial.print(bosKapMesafe_mm); Serial.println(F(" mm"));
    Serial.print(F("[KALIBRASYON] Maks su yuksekligi: ")); Serial.print(maxSeviye_mm);    Serial.println(F(" mm"));

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
      girisIndeksi     = 0;
      tuneSeciliParam  = 0;
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
    }
    else if (tus == '#') {
      uint16_t girilen = (uint16_t)atoi(girisBuffer);
      if (girilen > 0 && girilen <= maxSeviye_mm) {
        referans_mm = girilen;
        integralToplam_q8 = 0; // Q8 format sıfırla (windup temizle)
        // sonGercekSeviye: PID türev için y[k-1] başlatılıyor.
        // Mevcut su yüksekliğine eşitlenir → ilk turda türev şoku sıfır.
        sonGercekSeviye = anlik_su_yuksekligi;
        Serial.print(F("[HEDEF] ")); Serial.print(referans_mm); Serial.println(F(" mm"));
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
      // anlik_su_yuksekligi burada güncellenir → E_STOP ve PID taze veri kullanır,
      // döngü sonu ekranGuncelle de aynı taze veriyi görür.
      anlik_su_yuksekligi = (filtrelenmisMesafe_mm < bosKapMesafe_mm)
                            ? (bosKapMesafe_mm - filtrelenmisMesafe_mm) : 0;

      // Güvenlik 1: Sensör yanıt vermiyor (kablo kopması)
      if (filtrelenmisMesafe_mm == 0) {
        Serial.println(F("[E_STOP] Sensor yanit vermiyor!"));
        mevcutDurum = E_STOP;
        break;
      }

      // Güvenlik 2: Su taşması
      if (anlik_su_yuksekligi > maxSeviye_mm) {
        Serial.println(F("[E_STOP] Tasma tespit edildi!"));
        mevcutDurum = E_STOP;
        break;
      }

      // YAMA 2 & 3: Güvenlik Kontrolleri (PID sonrası, pompadan önce)
      //
      // Bu statik sayıç pid_zamaniGeldi bloğu içindedir:
      // stack değil BSS'de yaşar, her pid adımında hayatta kalır.
      static uint8_t stallSayaci = 0;

      // PID hesapla
      int16_t u = pidHesapla(referans_mm, anlik_su_yuksekligi);

      // YAMA 2: Kuru Çalışma Koruması (Dry-Run Protection)
      // Koşul: Su ≤ 5mm VE PID hala boşaltmaya çalışıyor (u < 0)
      // Tehlike: Boşaltım pompası havada kuru dönüyor → ısınma → yanma
      if (anlik_su_yuksekligi <= 5 && u < 0) {
        Serial.println(F("[E_STOP] Kuru calisma tespiti! Motor yanmasi engellendi."));
        mevcutDurum = E_STOP;
        break;
      }

      // YAMA 3: Tıkanıklık / Boru Kaçağı Tespiti (Stall Detection)
      // Koşul: Motor tam güe yakın (≥200 PWM) VE su seviyesi hiç değişmedi
      // Olası sebepler: Pompa tıkandı, boru koptu, valf kapandı
      // 50 döngü × 100ms = 5 saniye tepkisizlik → E_STOP
      if (abs(u) > 200 && anlik_su_yuksekligi == sonGercekSeviye) {
        stallSayaci++;
        if (stallSayaci >= 50) {
          Serial.println(F("[E_STOP] Motor tikanikligi veya boru kacagi tespiti!"));
          mevcutDurum = E_STOP;
          break;
        }
      } else {
        stallSayaci = 0; // Her şey normalişirse sayıcıyı sıfırla
      }

      aktif_pwm = u;
      pompaSur(u);

      Serial.print(F("H:")); Serial.print(referans_mm);
      Serial.print(F(" G:")); Serial.print(anlik_su_yuksekligi);
      Serial.print(F(" U:")); Serial.print(u);
      Serial.print(F(" STALL:")); Serial.println(stallSayaci);
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
    // PID KATSAYI AYAR MODU
    // =========================================================================
    // Giriş formatı: Değer ×10 olarak girilir.
    //   Örnek: Kp = 2.5 için "25" yaz, [#] bas.
    //   Dönüşüm: Q8 = (girilen_x10 × 256) / 10
    //
    // Aşama 1 (tuneParamSecildi=false): Hangi katsayı düzenlenecek?
    //   [1] Kp | [2] Ki | [3] Kd | [*] İptal → IDLE
    //
    // Aşama 2 (tuneParamSecildi=true): Değer girişi
    //   [0-9] Rakam | [A] Geri sil | [#] Onayla | [*] İptal → Aşama 1

    if (!tuneParamSecildi) {
      // --- Aşama 1: Katsayı seçimi ---
      if      (tus == '1') { tuneSeciliParam = 0; tuneParamSecildi = true; memset(girisBuffer,0,5); girisIndeksi = 0; }
      else if (tus == '2') { tuneSeciliParam = 1; tuneParamSecildi = true; memset(girisBuffer,0,5); girisIndeksi = 0; }
      else if (tus == '3') { tuneSeciliParam = 2; tuneParamSecildi = true; memset(girisBuffer,0,5); girisIndeksi = 0; }
      else if (tus == '*') { mevcutDurum = IDLE; }
    } else {
      // --- Aşama 2: Değer girişi ---
      if (tus >= '0' && tus <= '9') {
        if (girisIndeksi < 4) {
          girisBuffer[girisIndeksi++] = tus;
          girisBuffer[girisIndeksi]   = '\0';
        }
      } else if (tus == 'A' && girisIndeksi > 0) {
        girisBuffer[--girisIndeksi] = '\0';
      } else if (tus == '#') {
        uint16_t girilen_x10 = (uint16_t)atoi(girisBuffer);
        if (girilen_x10 > 0 && girilen_x10 <= 5000) { // 0.1 – 500.0 aralığı
          // Q8 dönüşümü: Q8 = (x10 × 256) / 10
          int16_t yeniQ8 = (int16_t)((int32_t)girilen_x10 * 256L / 10L);
          if      (tuneSeciliParam == 0) { Kp_q8 = yeniQ8;                   Serial.print(F("[TUNE] Kp=")); }
          else if (tuneSeciliParam == 1) { Ki_q8 = yeniQ8; integralToplam_q8 = 0; Serial.print(F("[TUNE] Ki=")); }
          else                           { Kd_q8 = yeniQ8;                   Serial.print(F("[TUNE] Kd=")); }
          Serial.print(girilen_x10 / 10); Serial.print('.'); Serial.println(girilen_x10 % 10);
          // Ekranı zorla güncelle: yeni değer statik çerçevede görünsün
          eski_durum = (SistemDurumu)0xFF;
          tuneParamSecildi = false;
          memset(girisBuffer, 0, 5);
          girisIndeksi = 0;
        } else {
          Serial.println(F("[TUNE] Gecersiz deger. 1-5000 arasi girin."));
          memset(girisBuffer, 0, 5);
          girisIndeksi = 0;
        }
      } else if (tus == '*') {
        // Aşama 2'den Aşama 1'e dön
        tuneParamSecildi = false;
        memset(girisBuffer, 0, 5);
        girisIndeksi = 0;
        eski_durum = (SistemDurumu)0xFF; // Hangi? mesajı yeniden çiz
      }
    }
    // TUNE_PID kendi ekranını yönetir (döngü sonu çağrısından hariç)
    if (mevcutDurum == TUNE_PID) {
      ekranGuncelle(TUNE_PID, girisIndeksi, tuneSeciliParam, (int16_t)tuneParamSecildi);
    }
    break;
  }

  case E_STOP: {
    analogWrite(DOLUM_POMPASI_PIN, 0);
    analogWrite(BOSALTIM_POMPASI_PIN, 0);
    aktif_pwm = 0;
    break;
  }

  } // Switch Sonu

  // Sadece SET_REFERENCE durumunda değilsek normal ekran güncellemesini çağır
  if (mevcutDurum != SET_REFERENCE && mevcutDurum != TUNE_PID) {
    ekranGuncelle(mevcutDurum, referans_mm, anlik_su_yuksekligi, aktif_pwm);
  }
}

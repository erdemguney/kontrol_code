// =============================================================================
// Kapalı Çevrim Su Seviyesi Kontrol Sistemi
// Mimari: Event-Driven State Machine (Olay-Güdümlü Durum Makinesi)
// =============================================================================
#include <Adafruit_GFX.h>    // Grafik temeli (Adafruit)
#include <Adafruit_ST7735.h> // ST7735S TFT sürücüsü
#include <Arduino.h>
#include <Keypad.h> // 4x4 matris tuş takımı
#include <SPI.h>    // ST7735 SPI veriyolu
#include <stdlib.h> // atoi()

// =============================================================================
// --- DONANIM SABİTLERİ ---
// =============================================================================
#define TRIGGER_PIN 3          // HC-SR04 Trigger → D3
#define ECHO_PIN 2             // HC-SR04 Echo    → D2 (INT0)
#define DOLUM_POMPASI_PIN 5    // Dolum Pompası   → D5 (Timer0, ~976 Hz)
#define BOSALTIM_POMPASI_PIN 6 // Boşaltım Pompası→ D6 (Timer0, ~976 Hz)
#define OLUBANT_ESIGI 40       // Ölü bant eşiği (PWM)

// --- ST7735S TFT SPI Pin Tanımları ---
// Hardware SPI: MOSI=D11, SCK=D13 (Arduino tarafından otomatik kullanılır)
#define TFT_CS_PIN 10 // Chip Select → D10
#define TFT_DC_PIN 9  // Data/Command → D9
#define TFT_RST_PIN 8 // Reset       → D8

// --- 4x4 Keypad Pin Tanımları ---
// Satır ve sütun pinleri analog giriş pinleri kullanılıyor (dijital olarak)
#define KP_SATIR_SAYISI 4
#define KP_SUTUN_SAYISI 4
#define PWM_MAX 255          // analogWrite() üst sınırı
#define INTEGRAL_LIMIT 255.0 // Anti-Windup kelepçe sınırı

// =============================================================================
// --- FAZ 6: HMI – EKRAN ve TUŞ TAKIMI NESNELERİ ---
// =============================================================================

// --- 6.1: ST7735S TFT Nesnesi ---
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

// --- 6.2: 4x4 Keypad Nesnesi ---
const char tusHaritasi[KP_SATIR_SAYISI][KP_SUTUN_SAYISI] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
byte satirPinleri[KP_SATIR_SAYISI] = {A0, A1, A2, A3}; // Satır pinleri
byte sutunPinleri[KP_SUTUN_SAYISI] = {A4, A5, 4, 7};   // Sütun pinleri
Keypad keypad = Keypad(makeKeymap(tusHaritasi), satirPinleri, sutunPinleri,
                       KP_SATIR_SAYISI, KP_SUTUN_SAYISI);

// --- 6.3: TFT Renk Sabitleri (RGB565) ---
#define RENK_ARKAPLAN 0x0000 // Siyah
#define RENK_YAZI 0xFFFF     // Beyaz
#define RENK_BASLIK 0x07FF   // Cyan   – durum başlıkları
#define RENK_ETIKET 0xFFE0   // Sarı   – "Hedef:", "Gercek:" sabit etiketler
#define RENK_DEGER 0x07E0    // Yeşil  – değişen sayısal veriler
#define RENK_DOLUM 0x001F    // Mavi   – dolum pompası aktif
#define RENK_BOSALTIM 0xF800 // Kırmızı– boşaltım pompası aktif / E_STOP

// --- 6.4: Delta-Update Durum Değişkenleri ---
// 0xFFFF = "Henüz yazılmadı" → ilk turda garantili güncelleme yapılır.
SistemDurumu eski_durum = (SistemDurumu)0xFF;
uint16_t eski_referans_mm = 0xFFFF;
uint16_t eski_gercekSeviye_mm = 0xFFFF;
int16_t eski_pwm = 32767;

// =============================================================================
// --- FAZ 6.5: ekranGuncelle() – Delta-Tabanlı Titremesiz HMI Güncelleyici ---
// =============================================================================
//
//  MANTIK:
//    • Durum değiştiyse  → ekranı tamamen sil, statik çerçeveyi çiz (1 kez)
//    • Değişken değiştiyse→ eski metnin üstüne siyah fillRect at, yenisini yaz
//    • Hiçbir şey değişmediyse → SPI'ye tek bit bile gönderilmez
//
void ekranGuncelle(SistemDurumu durum, uint16_t ref, uint16_t gercek,
                   int16_t pwm) {

  // -------------------------------------------------------------------------
  // ADIM 1: Durum değişimi – statik çerçeveyi bir kez çiz
  // -------------------------------------------------------------------------
  if (durum != eski_durum) {
    tft.fillScreen(RENK_ARKAPLAN); // Ekranı tamamen sil (sadece geçişte!)

    switch (durum) {

    case INIT_CALIBRATION:
      tft.setTextColor(RENK_BASLIK, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(8, 8);
      tft.print(F("KALIBRASYON"));
      tft.drawFastHLine(0, 20, tft.width(), RENK_BASLIK);
      tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
      tft.setCursor(8, 30);
      tft.print(F("Lutfen bekleyin..."));
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
      // Sabit etiketler – hiç değişmeyecek, tek seferde çizilir
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
    // Durum geçişinde delta değişkenlerini sıfırla →
    // aşağıdaki karşılaştırmalar ilk turda mutlaka yazacak.
    eski_referans_mm = 0xFFFF;
    eski_gercekSeviye_mm = 0xFFFF;
    eski_pwm = 32767;
  }

  // -------------------------------------------------------------------------
  // ADIM 2: Dinamik alanlar – SADECE değişim varsa güncelle
  //         fillRect → eski metni sil → yeni metni yaz
  // -------------------------------------------------------------------------

  if (durum == SET_REFERENCE) {
    // ref yerine girisIndeksi'ni taşıyoruz: eşsiz değişim algıla
    if (ref != eski_referans_mm) {
      tft.fillRect(8, 44, 120, 14, RENK_ARKAPLAN); // Eski girişi sil
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(8, 46);
      if (girisIndeksi == 0) {
        tft.print(F("_")); // Henüz giriş yok
      } else {
        tft.print(girisBuffer); // Yazılan karakterler
        tft.print(F(" mm"));
      }
      eski_referans_mm = ref;
    }
  } else if (durum == AUTO_CONTROL) {

    // 2a. Hedef su yüksekliği
    if (ref != eski_referans_mm) {
      tft.fillRect(60, 30, 68, 12, RENK_ARKAPLAN);
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(60, 30);
      tft.print(ref);
      tft.print(F(" mm"));
      eski_referans_mm = ref;
    }

    // 2b. Gerçek su yüksekliği
    if (gercek != eski_gercekSeviye_mm) {
      tft.fillRect(60, 46, 68, 12, RENK_ARKAPLAN);
      tft.setTextColor(RENK_DEGER, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(60, 46);
      tft.print(gercek);
      tft.print(F(" mm"));
      eski_gercekSeviye_mm = gercek;
    }

    // 2c. PWM / Pompa yönü
    if (pwm != eski_pwm) {
      tft.fillRect(60, 62, 68, 12, RENK_ARKAPLAN);
      tft.setTextSize(1);
      tft.setCursor(60, 62);
      if (pwm > 0) {
        tft.setTextColor(RENK_DOLUM, RENK_ARKAPLAN); // Mavi = doldur
        tft.print('+');
        tft.print(pwm);
        tft.print(F(" D"));
      } else if (pwm < 0) {
        tft.setTextColor(RENK_BOSALTIM, RENK_ARKAPLAN); // Kırmızı = boşalt
        tft.print(pwm);
        tft.print(F(" B"));
      } else {
        tft.setTextColor(RENK_YAZI, RENK_ARKAPLAN);
        tft.print(F("OLUBANT"));
      }
      eski_pwm = pwm;
    }
  }
}

// =============================================================================
// FAZ 1: ISR – HC-SR04 Yankı Süresi Ölçümü
// =============================================================================
volatile unsigned long yankibaslangicZamani = 0;
volatile unsigned long yankiSuresi = 0;
volatile bool yeniVeriHazir = false;

void echo_ISR() {
  if (PIND & (1 << PD2)) { // Yükselen kenar
    yankibaslangicZamani = micros();
  } else { // Düşen kenar
    yankiSuresi = micros() - yankibaslangicZamani;
    yeniVeriHazir = true;
  }
}

// =============================================================================
// FAZ 2: Veri İşleme – Ring Buffer + Medyan Filtresi + Low-Pass Filtre
// =============================================================================
uint16_t sonOlcumler[5] = {0, 0, 0, 0, 0};
uint8_t olcumIndeksi = 0;
uint16_t filtrelenmisMesafe_mm = 0;

void veriyiIsle(unsigned long yanki) {
  // 1. µs → mm (float kullanmadan): mm = (yanki * 10) / 58
  uint16_t anlikMesafe_mm = (uint16_t)((yanki * 10UL) / 58UL);

  // 2. Ring Buffer
  sonOlcumler[olcumIndeksi] = anlikMesafe_mm;
  olcumIndeksi = (olcumIndeksi + 1) % 5;

  // 3. Bubble Sort → Medyan
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

  // 4. IIR Low-Pass Filtre (alpha = 1/4, float yok)
  filtrelenmisMesafe_mm = (filtrelenmisMesafe_mm * 3 + medyan) / 4;
}

// =============================================================================
// FAZ 3: Eyleyici Kontrolü – HAL (Deadband + Clamping + Interlock)
// =============================================================================
void pompaSur(int16_t kontrolSinyali) {
  // Kural 3: Ölü Bant
  if (kontrolSinyali > -OLUBANT_ESIGI && kontrolSinyali < OLUBANT_ESIGI) {
    analogWrite(DOLUM_POMPASI_PIN, 0);
    analogWrite(BOSALTIM_POMPASI_PIN, 0);
    return;
  }
  // Kural 2: Clamping
  int16_t mutlak = abs(kontrolSinyali);
  uint8_t pwmDeger = (mutlak > PWM_MAX) ? (uint8_t)PWM_MAX : (uint8_t)mutlak;

  // Kural 1: Interlock (önce kapat, sonra aç)
  if (kontrolSinyali > 0) {
    analogWrite(BOSALTIM_POMPASI_PIN, 0);     // Boşaltımı kapat
    analogWrite(DOLUM_POMPASI_PIN, pwmDeger); // Dolumu aç
  } else {
    analogWrite(DOLUM_POMPASI_PIN, 0);           // Dolumu kapat
    analogWrite(BOSALTIM_POMPASI_PIN, pwmDeger); // Boşaltımı aç
  }
}

// =============================================================================
// FAZ 4: Ayrık Zamanlı PID Kontrolcüsü
//   • Derivative on Measurement → Türevsel Şok engeli
//   • Integral Clamping         → Anti-Windup
// =============================================================================
float Kp = 2.0;
float Ki = 0.5;
float Kd = 1.0;
float integralToplam = 0.0;   // Integral biriktirici
uint16_t sonGercekSeviye = 0; // y[k-1] – önceki ölçüm
const float dt = 0.1;         // Timer1: 10 Hz → dt = 0.1 s

int16_t pidHesapla(uint16_t referans_mm, uint16_t gercekSeviye_mm) {
  // 1. Hata
  float e = (float)referans_mm - (float)gercekSeviye_mm;
  // 2. P
  float P = Kp * e;
  // 3. I + Anti-Windup
  integralToplam += Ki * e * dt;
  if (integralToplam > INTEGRAL_LIMIT)
    integralToplam = INTEGRAL_LIMIT;
  else if (integralToplam < -INTEGRAL_LIMIT)
    integralToplam = -INTEGRAL_LIMIT;
  float I = integralToplam;
  // 4. D – Derivative on Measurement (referans şoku yok)
  //    D = -Kd * (y[k] - y[k-1]) / dt
  float D = -Kd * ((float)gercekSeviye_mm - (float)sonGercekSeviye) / dt;
  // 5. Toplam
  int16_t u = (int16_t)(P + I + D);
  // 6. Durum güncelle
  sonGercekSeviye = gercekSeviye_mm;
  return u;
}

// =============================================================================
// FAZ 5: Event-Driven Durum Makinesi (State Machine)
// =============================================================================

// --- 5.1: Durum Tanımı ---
enum SistemDurumu {
  INIT_CALIBRATION, // Açılışta sensör kalibrasyonu
  IDLE,             // Bekleme – motorlar durur
  SET_REFERENCE,    // Kullanıcı hedef seviyeyi girer
  AUTO_CONTROL,     // PID döngüsü aktif
  E_STOP            // Acil durdurma – reset bekler
};

volatile SistemDurumu mevcutDurum = INIT_CALIBRATION;

// --- 5.2: Sistem Değişkenleri ---
uint16_t referans_mm =
    0; // Kullanıcının girdiği hedef su YÜKSEKLİĞİ (mm, dipten)
uint16_t bosKapMesafe_mm =
    0; // Kalibrasyon: boş kaptaki sensör MESAFESİ (mm, sensörden dibe)
uint16_t maxSeviye_mm =
    0; // Fiziksel maksimum doldurulabilir su YÜKSEKLİĞİ (mm, dipten)
//
// KOORDİNAT SİSTEMİ:
//   bosKapMesafe_mm  → MESAFE (sensörden aşağı, büyük = boş kap)
//   maxSeviye_mm     → YÜKSEKLİK (dipten yukarı, büyük = dolu kap)
//   Dönüşüm: suYuksekligi = bosKapMesafe_mm - filtrelenmisMesafe_mm
// SET_REFERENCE: kullanıcı tuş girişini biriktiren STATİK karakter dizisi.
// String sınıfı YASAK – heap fragmentation yapar, 2KB SRAM'ı öldürür.
// Max 4 hane + null terminator = 5 byte, yığın belleğinde sabit durur.
char girisBuffer[5] = {0}; // '\0' ile dolu başlangıç (null-terminated C string)
uint8_t girisIndeksi = 0;  // Buffer'ın o anki yazma konumu

// --- 5.3: Timer1 Bayrağı ---
// ISR bu bayrağı set eder; loop() okur ve temizler.
volatile bool pid_zamaniGeldi = false;

// aktif_pwm: AUTO_CONTROL'de pidHesapla()'dan çıkan son kontrol sinyali.
// loop() sonundaki ekranGuncelle() bu global'i okur.
// Diğer durumlarda 0 kalır (motorlar durmuş, ölü bant).
int16_t aktif_pwm = 0;

// --- 5.4: Timer1 ISR (CTC, 100ms = 10 Hz) ---
ISR(TIMER1_COMPA_vect) {
  if (mevcutDurum == AUTO_CONTROL) {
    // HC-SR04 Trigger darbesi (10 µs)
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    // Ana döngüyü bilgilendir
    pid_zamaniGeldi = true;
  }
}

// --- 5.5: setup() ---
void setup() {
  Serial.begin(9600);

  // Pin modları
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(DOLUM_POMPASI_PIN, OUTPUT);
  pinMode(BOSALTIM_POMPASI_PIN, OUTPUT);

  // Güvenli başlangıç: her iki pompayı kapat
  analogWrite(DOLUM_POMPASI_PIN, 0);
  analogWrite(BOSALTIM_POMPASI_PIN, 0);

  // Echo kesmesi: D2 (INT0), hem yükselen hem düşen kenar
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echo_ISR, CHANGE);

  // Timer1 CTC Modu: 100ms periyot (10 Hz)
  // OCR1A = (16,000,000 / 1024 / 10) - 1 = 1561
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1561;
  TCCR1B |= (1 << WGM12);              // CTC modu
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler: 1024
  TIMSK1 |= (1 << OCIE1A);             // Compare Match kesmesi aktif
  interrupts();

  // --- TFT Başlatma ---
  tft.initR(INITR_BLACKTAB); // ST7735S için siyah tab tipi
  tft.setRotation(1);        // Yatay mod (160x128)
  tft.fillScreen(RENK_ARKAPLAN);
  tft.setTextWrap(false); // Metin taşmasını engelle

  // --- Keypad Debounce Ayarı ---
  // Varsayılan 10ms – PID döngümüzü bloklamaz, bounce’a karşı yeterli
  keypad.setDebounceTime(10);

  // lcd.init(); lcd.backlight(); // Kaldırıldı – ST7735 kullanılıyor

  // İlk ekranı çiz (INIT_CALIBRATION)
  ekranGuncelle(INIT_CALIBRATION, 0, 0, 0);
}

// =============================================================================
// --- 5.6: loop() – Ana Durum Makinesi ---
// =============================================================================
void loop() {
  // --- Keypad: Non-Blocking Okuma ---
  // getKey() bloklamaz; tuş yoksa NO_KEY (0) döndürür.
  // Debounce Keypad kütüphanesi içinde yönetilir.
  char tus = keypad.getKey();
  if (tus == NO_KEY)
    tus = 0; // 0 = tuş yok

  switch (mevcutDurum) {

  // =========================================================================
  case INIT_CALIBRATION:
    // =========================================================================
    // Cihaz açıldığında bir kez çalışır.
    // Sensör boş kabı ölçer → bosSeviye_mm ve maxSeviye_mm hesaplanır.
    // Biter bitmez otomatik olarak IDLE'a geçilir.
    {
      // lcd.clear(); lcd.print("Kalibrasyon...");
      Serial.println(F("Kalibrasyon basliyor..."));

      uint32_t toplam = 0;
      const uint8_t N = 10; // 10 ölçümün ortalaması alınır

      for (uint8_t i = 0; i < N; i++) {
        // Trigger darbesi
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);
        delay(60); // HC-SR04 için ölçümler arası minimum bekleme

        noInterrupts();
        unsigned long yanki = yankiSuresi;
        interrupts();

        veriyiIsle(yanki);
        toplam += filtrelenmisMesafe_mm;
      }

      bosKapMesafe_mm = (uint16_t)(toplam / N);

      // Sensör ölü bölgesi: HC-SR04 ~20 mm altını ölçemez (kör nokta).
      // Bu, bardağın dolabileceği maksimum su YÜKSEKLİĞİni belirler.
      // maxSeviye_mm = bosKapMesafe_mm - kör_nokta  (YÜKSEKLİK birimi, dipten!)
      const uint16_t SENSOR_KOR_NOKTASI_MM = 20;
      maxSeviye_mm = (bosKapMesafe_mm > SENSOR_KOR_NOKTASI_MM)
                         ? (bosKapMesafe_mm - SENSOR_KOR_NOKTASI_MM)
                         : 0;

      Serial.print(F("Bos kap MESAFESI  : "));
      Serial.print(bosKapMesafe_mm);
      Serial.println(F(" mm  [sensorden dibe]"));
      Serial.print(F("Maks su YUKSEKLIGI: "));
      Serial.print(maxSeviye_mm);
      Serial.println(F(" mm  [dipten yukari]"));

      // lcd.clear(); lcd.print("Maks:"); lcd.print(maxSeviye_mm);
      // lcd.print("mm");
      mevcutDurum = IDLE; // Otomatik geçiş
      break;
    }

  // =========================================================================
  case IDLE:
    // =========================================================================
    // Motorlar durur. Kullanıcının keypad'e basmasını bekler.
    // '*' tuşu → SET_REFERENCE
    {
      pompaSur(0); // Motorları güvenle durdur

      // lcd.setCursor(0,0); lcd.print("BEKLENIYOR      ");
      // lcd.setCursor(0,1); lcd.print("*: Seviye Gir   ");

      if (tus == '*') {
        // Buffer'i sıfırla: tüm byte'ları 0'a çek, indeksi başa al
        memset(girisBuffer, 0, sizeof(girisBuffer));
        girisIndeksi = 0;
        // lcd.clear(); lcd.print("Hedef (mm):");
        mevcutDurum = SET_REFERENCE;
      }
      break;
    }

  // =========================================================================
  case SET_REFERENCE:
    // =========================================================================
    // Kullanıcı rakam tuşlarıyla hedef su seviyesini (mm) girer.
    // '#' → Onayla ve AUTO_CONTROL'e geç
    // '*' → İptal, IDLE'a dön
    // NOT: Bu esnada PID hesaplanmaz, sistem güvende.
    {
      if (tus >= '0' && tus <= '9') {
        if (girisIndeksi < 4) {             // Max 4 hane (9999 mm)
          girisBuffer[girisIndeksi] = tus;  // Karakteri diziye yaz
          girisIndeksi++;                   // Sonraki konuma geç
          girisBuffer[girisIndeksi] = '\0'; // Null-terminator'u güncelle
          // lcd.setCursor(0,1); lcd.print(girisBuffer);
          Serial.print(F("Giris: "));
          Serial.println(girisBuffer);
        }
      } else if (tus == '#') { // Onayla
        // atoi(): stdlib.h içinde, null-terminated C string'i int'e çevirir.
        // String.toInt() tersine heap kullanmaz, yığında çalışır.
        uint16_t girilen = (uint16_t)atoi(girisBuffer);

        if (girilen > 0 && girilen <= maxSeviye_mm) {
          referans_mm = girilen;

          // Geçiş öncesi PID durumunu sıfırla (windup temizle)
          integralToplam = 0.0;
          // ÖNEMLİ: sonGercekSeviye, PID türevi için y[k-1] değeridir.
          // filtrelenmisMesafe_mm bir MESAFE'dir; bunu YÜKSEKLİĞE çeviriyoruz.
          sonGercekSeviye = (filtrelenmisMesafe_mm < bosKapMesafe_mm)
                                ? (bosKapMesafe_mm - filtrelenmisMesafe_mm)
                                : 0;

          Serial.print(F("Hedef seviye: "));
          Serial.print(referans_mm);
          Serial.println(F(" mm"));
          // lcd.clear(); lcd.print("OTO KONTROL");
          mevcutDurum = AUTO_CONTROL;
        } else {
          // Geçersiz değer: buffer'i sıfırla, kullanıcı tekrar girsin
          Serial.println(F("GECERSIZ DEGER! Tekrar girin."));
          // lcd.setCursor(0,1); lcd.print("GECERSIZ!       ");
          memset(girisBuffer, 0, sizeof(girisBuffer));
          girisIndeksi = 0;
        }
      } else if (tus == '*') { // İptal
        memset(girisBuffer, 0, sizeof(girisBuffer));
        girisIndeksi = 0;
        mevcutDurum = IDLE;
      }
      break;
    }

  // =========================================================================
  case AUTO_CONTROL:
    // =========================================================================
    // Timer1 kesmesi (100ms) pid_zamaniGeldi bayrağını set eder.
    // Bayrak görüldüğünde: sensör okunur → PID hesaplanır → pompa sürülür.
    // '*' tuşu → IDLE'a dön (motorlar durur)
    {
      if (pid_zamaniGeldi) {
        pid_zamaniGeldi =
            false; // Bayrağı hemen temizle (sonraki kesmeyi kaçırma)

        // ISR'dan atomik veri kopyası
        noInterrupts();
        unsigned long yanki = yankiSuresi;
        interrupts();

        // Sensör verisini işle → filtrelenmisMesafe_mm güncellenir
        veriyiIsle(yanki);

        // --- Güvenlik Kontrolü (E_STOP koşulları) ---
        // Koşul 1: Sensör yanıt yok (kablo kopması) → değer 0 kalır
        if (filtrelenmisMesafe_mm == 0) {
          Serial.println(F("E_STOP: Sensor yanit vermiyor!"));
          mevcutDurum = E_STOP;
          break;
        }

        // Koşul 2: Taşma kontrolü
        // KOORDİNAT DÖNÜŞÜMÜ: Mesafe → Yükseklik
        //   Sensör MESAFE'yi ölçer (büyük = boş, küçük = dolu).
        //   PID ve güvenlik mantığı su YÜKSEKLİĞİ'ne ihtiyaç duyar (büyük =
        //   dolu). Dönüşüm: suYuksekligi_mm = bosKapMesafe_mm -
        //   filtrelenmisMesafe_mm
        uint16_t gercekSeviye_mm = 0;
        if (filtrelenmisMesafe_mm < bosKapMesafe_mm) {
          gercekSeviye_mm = bosKapMesafe_mm - filtrelenmisMesafe_mm;
        }
        // Taşma: su yüksekliği fiziksel maksimumu geçtiyse E_STOP
        // (Mesafe sensörün kör noktasına yaklaştı = kap doldu aştı)
        if (gercekSeviye_mm > maxSeviye_mm) {
          Serial.println(F("E_STOP: Su yuksekligi fiziksel siniri asti!"));
          mevcutDurum = E_STOP;
          break;
        }

        // --- PID Hesapla → Pompayı Sür ---
        int16_t u = pidHesapla(referans_mm, gercekSeviye_mm);
        aktif_pwm =
            u; // Ekran için global'e kaydet (loop sonu ekranGuncelle okuyacak)
        pompaSur(u);

        // Seri port debug (isteğe bağlı)
        Serial.print(F("H:"));
        Serial.print(referans_mm);
        Serial.print(F(" G:"));
        Serial.print(gercekSeviye_mm);
        Serial.print(F(" U:"));
        Serial.println(u);

        // lcd.setCursor(0,0); lcd.print("H:"); lcd.print(referans_mm);
        // lcd.print("mm   "); lcd.setCursor(0,1); lcd.print("G:");
        // lcd.print(gercekSeviye_mm); lcd.print("mm   ");
      }

      // Kullanıcı çıkış tuşuna bastıysa IDLE'a dön
      if (tus == '*') {
        pompaSur(0); // Anında motorları durdur
        mevcutDurum = IDLE;
      }
      break;
    }

  // =========================================================================
  case E_STOP:
    // =========================================================================
    // Kritik hata! Buradan çıkmak için fiziksel RESET gerekir.
    // Motorlar her döngüde sıfırlanır (parazit sinyale karşı savunma).
    {
      analogWrite(DOLUM_POMPASI_PIN, 0);
      analogWrite(BOSALTIM_POMPASI_PIN, 0);

      // lcd.setCursor(0,0); lcd.print("!!! HATA !!!    ");
      // lcd.setCursor(0,1); lcd.print("RESET GEREKLI   ");

      // Başka hiçbir şey yapma. Fiziksel RESET basılana kadar burada kal.
      break;
    }

  } // switch sonu

  // =========================================================================
  // --- Ekran Güncelleme (loop'un EN SONUNDA, switch-case'den sonra) ---
  // =========================================================================
  // aktif_pwm: switch-case içinde AUTO_CONTROL'de güncellenen global değişken.
  // Diğer tüm durumlarda 0 olarak kalır.
  {
    uint16_t anlikSeviye = (filtrelenmisMesafe_mm < bosKapMesafe_mm)
                               ? (bosKapMesafe_mm - filtrelenmisMesafe_mm)
                               : 0;
    ekranGuncelle(mevcutDurum, referans_mm, anlikSeviye, aktif_pwm);
  }
}

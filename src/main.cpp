#include <Arduino.h>
#include <STM32F1_RTC.h>
#include <EEPROM.h>
#include <OneWire.h>

int calisma_sayac_sifirla = 0;

/* Saat Ayarlar */
#define FLAGS_REGISTER 1
#define HOUR_12_BIT 2
#define HOUR_12_FLAG (1 << HOUR_12_BIT)
#define IS_HOUR_12(x) ((x.getBackupRegister(FLAGS_REGISTER) & HOUR_12_FLAG) == HOUR_12_FLAG)
#define TOGGLE_HOUR_12(x) (x.setBackupRegister(FLAGS_REGISTER, x.getBackupRegister(FLAGS_REGISTER) ^ HOUR_12_FLAG))

// STM32 PIN Setup
#define Pwm PB9
#define akim_sensor PA4
#define akis_sensor PA7
#define volt_sensor PA0
#define DS18B20_sens PB5 // DS18B20 su sıcaklığı sensörü (UA1 soketi)
#define V5_sensor PA1    // işlemci besleme voltajı
#define filtre_pompa PB0
#define havuz_isik PB12
#define role_1 PB13
#define role_2 PB14
#define role_3 PB15
#define role_4 PA8
#define bilgi_led PC13
#define WINDOW_SIZE_A 50 // akım filtre örnek sayısı
#define BACKWASH_ARRAY_SIZE 5
#define POLARITE_TIMER 5 // Polarite degisim zamani saat
#define PWM_MINIMUM 3750 // MT-250-12 icin minimum PWM degeri

int READINGS_A[WINDOW_SIZE_A];
int akim_index = 0;
int sum_akim = 0;
int ortalama_akim = 0;
int polarite = 1;
int polariteEprom = 0;
int stab = 0;
int setup_akim = 0;
int polarite_timer_hour = 0;

byte cepGelen[6] = {0};
int byte_uzunluk = 0;
int byte_kontrol = 0;

int backwash_values[BACKWASH_ARRAY_SIZE];
int backwash_valueIndex = 0;
int backwash_saniye = 0;
int backwash_median = 0;
int backwash_max = 0;
int backwash_rpm = 0;
int polarite_ikon_guncelle = 0;
int guc_acik = 0;
int backwash_median_resim = 2272;

/* Debug ayarlari */
int akis_debug = 0;  // Akis Sensoru Debug
int saat_debug = 0;  // Saat Debug icin 1 yapilacak
int ekran_debug = 0; // LCD gelen data debug
int akim_debug = 0;  // Akim debug icin
int eprom_debug = 0; // Eprom degerleri debug icin
int genel_debug = 1; // Genel Durum Debug0
/*Saat Degiskenleri*/
uint32_t epochTime;
uint32_t epochDate;
int isHour12 = 0;
DateVar date;
TimeVar time;
TimeVar time_save;
STM32F1_RTC rtc;

OneWire oneWire(PA6);

float sicaklik = 0.0;
float sicaklik_eski = 0.0;
int filtre_durum_sicaklik = 0;
float sicaklik_1 = 60.0;
float sicaklik_2 = 70.0;

int akim_pwm = 0;
float voltaj_girdi = 0;
int uretim_hedef = 0;      // Ekrandan gelen Hedeflenen Uretim
int istenen_akim = 0;      // Hedeflenen akimin pwm degeri
int ekran_uretim = 0;      // Ekrana gonderilecek electro degeri
int ekran_uretim_eski = 0; // Ekrana gonderilecek electro degeri saklama
int akim_deger = 0;        // Akim Sensorunden okunan deger
int minimum_akim = 2063;   // Sistemin minimum akim degeri = 2047
float akim_katsayi = 14;   // Uretim hesaplama katsayisi = 16.4
int kapasite = 50;         // Sistemin uretim max kapasitesi
int eprom_deger;
int pwm_Max = 7250; // MAX PWM degeri
int akim_anlik = 0;
int akim_toplama_sayac = 0;
int filtre_acik = 0;

uint16 filtre_t1_s_eprom;
uint16 filtre_t1_e_eprom;
uint16 filtre_t2_s_eprom;
uint16 filtre_t2_e_eprom;
uint16 isik_t1_s_eprom;
uint16 isik_t1_e_eprom;
uint16 isik_t2_s_eprom;
uint16 isik_t2_e_eprom;
uint16 aux1_t1_s_eprom;
uint16 aux1_t1_e_eprom;
uint16 aux1_t2_s_eprom;
uint16 aux1_t2_e_eprom;
uint16 aux2_t1_s_eprom;
uint16 aux2_t1_e_eprom;
uint16 aux2_t2_s_eprom;
uint16 aux2_t2_e_eprom;

int backwash_deger = 0;        // Su akis sensor degeri
int backwash_deger_onceki = 0; // Su akis deger saklama
int backwash_hiz = 0;
int backwash_hiz_onceki = 0;
int akis_enable = 1;
int saat_guncelle = 0;
int filtre_timer_guncelle = 0;
int isik_timer_guncelle = 0;
int aux1_timer_guncelle = 0;
int aux2_timer_guncelle = 0;
int ekran_ayar_saat_set = 0;
int uretim_hedef_guncelle = 0;

// Ekran okuma - yazma degiskeleri
unsigned char ekran_okuma[7] = {0X5A, 0XA5, 0X04, 0X83, 0x20, 0x00, 0X01};
byte endian[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
byte endian2[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
byte endian4[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned char ekran_gelen[80];                                                                            // Ekrandan gelen katar icin buffer
int ekran_buffer = 0;                                                                                     // buffer index
int baslik_1, baslik_2, adres_1, adres_2, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8; // Katar baslik, adress ve data
unsigned char ekrana_yolla[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00};
unsigned char ekran_sayfa[11] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01};
unsigned char backwash[8] = {0x5A, 0xA5, 0x05, 0x82, 0x22, 0x72, 0x00, 0x00};
unsigned char color[8] = {0x5A, 0xA5, 0x05, 0x82, 0x90, 0x03, 0xF8, 0x00};

// Zaman degiskenleri
int saat = 0;
int dakika = 0;
int saniye = 0;
int gun = 0;
int ay = 0;
int yil = 0;
int millis_sakla = 0;
int akis_millis_sakla = 0;
int akim_millis_sakla = 0;
int backwash_millis = 0;
int backwash_counter = 0;
int millis_250 = 0;
int zaman_cek = 0;         // Ekrandan set saat guncelle?
int zaman_sayac = 0;       // Ekrandan set zaman cekme sayaci
int ekran_yil = 0;         // Ekrandan gelen yil
int ekran_ay = 0;          // Ekrandan gelen ay
int ekran_gun = 0;         // Ekrandan gelen gun
int ekran_saat = 0;        // Ekrandan gelen saat
int ekran_dakika = 0;      // Ekrandan gelen dakika
int hesaplanan_dakika = 0; // Mevcut gunun dakika degeri
int filtre_timer_1_sh = 0; // 1. Filtrasyon Zamanlayici Baslangic saati
int filtre_timer_1_sm = 0; // 1. Filtrasyon Zamanlayici Baslangic dakikasi
int filtre_timer_1_eh = 0; // 1. Filtrasyon Zamanlayici Bitis saati
int filtre_timer_1_em = 0; // 1. Filtrasyon Zamanlayici Bitis dakikasi
int filtre_timer_2_sh = 0; // 2. Filtrasyon Zamanlayici Baslangic saati
int filtre_timer_2_sm = 0; // 2. Filtrasyon Zamanlayici Baslangic dakikasi
int filtre_timer_2_eh = 0; // 2. Filtrasyon Zamanlayici Bitis saati
int filtre_timer_2_em = 0; // 2. Filtrasyon Zamanlayici Bitis dakikasi
int filtre_baslama_1 = 0;  // 1. Filtrasyon Zamanlayici Baslangic Gunun dakika degeri
int filtre_bitme_1 = 0;    // 1. Filtrasyon Zamanlayici Bitis Gunun dakika degeri
int filtre_baslama_2 = 0;  // 2. Filtrasyon Zamanlayici Baslangic Gunun dakika degeri
int filtre_bitme_2 = 0;    // 2. Filtrasyon Zamanlayici Bitis Gunun dakika degeri

int isik_timer_1_sh = 0;
int isik_timer_1_sm = 0;
int isik_timer_1_eh = 0;
int isik_timer_1_em = 0;
int isik_timer_2_sh = 0;
int isik_timer_2_sm = 0;
int isik_timer_2_eh = 0;
int isik_timer_2_em = 0;
int isik_baslama_1 = 0;
int isik_bitme_1 = 0;
int isik_baslama_2 = 0;
int isik_bitme_2 = 0;

int aux1_timer_1_sh = 0;
int aux1_timer_1_sm = 0;
int aux1_timer_1_eh = 0;
int aux1_timer_1_em = 0;
int aux1_timer_2_sh = 0;
int aux1_timer_2_sm = 0;
int aux1_timer_2_eh = 0;
int aux1_timer_2_em = 0;
int aux1_baslama_1 = 0;
int aux1_bitme_1 = 0;
int aux1_baslama_2 = 0;
int aux1_bitme_2 = 0;

int aux2_timer_1_sh = 0;
int aux2_timer_1_sm = 0;
int aux2_timer_1_eh = 0;
int aux2_timer_1_em = 0;
int aux2_timer_2_sh = 0;
int aux2_timer_2_sm = 0;
int aux2_timer_2_eh = 0;
int aux2_timer_2_em = 0;
int aux2_baslama_1 = 0;
int aux2_bitme_1 = 0;
int aux2_baslama_2 = 0;
int aux2_bitme_2 = 0;

int low_alarm = 0;
int low_alarm_eski = 0;
int flow_alarm = 0;
int flow_alarm_eski = 0;
int backwash_alarm = 0;
int backwash_alarm_eski = 0;

int filtre_durum = 0;      // Filtre anlik durum
int filtre_durum_eski = 0; // Filtre onceki durum
int isik_durum = 0;        // Isik anlik durum
int isik_durum_eski = 0;   // Isik onceki durum
int aux1_durum = 0;        // Aux-1 anlik durum
int aux1_durum_eski = 0;   // Aux-1 onceki durum
int aux2_durum = 0;        // Aux-2 anlik durum
int aux2_durum_eski = 0;   // Aux-2 onceki durum
int backwash_durum = 0;
int backwash_mod = 0;
int polarite_durum = 0;      // Ikon durum
int polarite_durum_eski = 4; // Ikon durum sakla

long calisma_sayac = 0;
int calisma_saat = 0;
int calisma_eprom = 0;
int dil = 0; // English
int saat_gonder = 1;
int backwash_anlik = 0;

int akis_hiz = 0;
int akis_hiz_onceki = 0;
int akis_deger = 0;
int akis_anlik = 0;
int akis_deger_onceki = 0;
int akis_saklama = 0;

void update_version()
{
  unsigned char versiyon[14] = {0x5A, 0xA5, 0x0B, 0x82, 0x22, 0x58, 0x41, 0x54, 0x2D, 0x42, 0x53, 0x2D, 0x31, 0x35};
  Serial1.write(versiyon, 14);
}

void eprom_donustur()
{
  int deger = (filtre_durum * 10000) + (isik_durum * 1000) + (aux1_durum * 100) + (aux2_durum * 10) + (dil);
  EEPROM.update(10, deger);
}

void komut_olustur(int deger)
{
  memcpy(endian, &deger, 2);
  ekrana_yolla[6] = endian[1];
  ekrana_yolla[7] = endian[0];
}

void role_kontrol_set(int role, int durum)
{
  if (digitalRead(role) == 1)
  {
    if (durum == 0)
    {
      digitalWrite(role, LOW);
    }
  }
  else
  {
    if (durum == 1)
    {
      digitalWrite(role, HIGH);
    }
  }
}

void polarite_set()
{
  if (polarite == 1)
  {
    polarite_durum = 1;
  }
  else if (polarite == 2)
  {
    polarite_durum = 3;
  }
}

void backwash_anlik_ekrana_gonder(int tim)
{
  memcpy(endian, &tim, 3);
  unsigned char calisma[8] = {0x5A, 0xA5, 0x05, 0x82, 0x22, 0x70, (unsigned char)endian[1], (unsigned char)endian[0]};
  Serial1.write(calisma, 8);
}

void calisma_saati_guncelle(int tim)
{
  memcpy(endian, &tim, 3);
  unsigned char calisma[8] = {0x5A, 0xA5, 0x05, 0x82, 0x22, 0x57, (unsigned char)endian[1], (unsigned char)endian[0]};
  Serial1.write(calisma, 8);
}

void ekran_sayfa_degistir()
{
  Serial1.write(ekran_sayfa, 11);
}

void backwash_alarm_guncelle()
{
  if (backwash_alarm != backwash_alarm_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x04;
    komut_olustur(78 + backwash_alarm);
    Serial1.write(ekrana_yolla, 8);
    backwash_alarm_eski = backwash_alarm;
  }
}
void filtre_oto_kontrol() // Filtre oto timer kontrol
{
  filtre_baslama_1 = (filtre_timer_1_sh * 60) + filtre_timer_1_sm;
  filtre_bitme_1 = (filtre_timer_1_eh * 60) + filtre_timer_1_em;
  filtre_baslama_2 = (filtre_timer_2_sh * 60) + filtre_timer_2_sm;
  filtre_bitme_2 = (filtre_timer_2_eh * 60) + filtre_timer_2_em;

  if ((filtre_baslama_1 == filtre_bitme_1) && (filtre_baslama_2 == filtre_bitme_2))
  {
    filtre_durum = 2;
  }
  else if ((filtre_baslama_1 < filtre_bitme_1) && (filtre_baslama_2 == filtre_bitme_2))
  {
    if ((filtre_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < filtre_bitme_1))
    {
      filtre_durum = 3;
    }
    else
    {
      filtre_durum = 2;
    }
  }
  else if ((filtre_baslama_1 > filtre_bitme_1) && (filtre_baslama_2 == filtre_bitme_2))
  {
    if (hesaplanan_dakika < filtre_baslama_1 && hesaplanan_dakika >= filtre_bitme_1)
    {
      filtre_durum = 2;
    }
    else
    {
      filtre_durum = 3;
    }
  }
  else if ((filtre_baslama_2 < filtre_bitme_2) && (filtre_baslama_1 == filtre_bitme_1))
  {
    if ((filtre_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < filtre_bitme_2))
    {
      filtre_durum = 3;
    }
    else
    {
      filtre_durum = 2;
    }
  }
  else if ((filtre_baslama_2 > filtre_bitme_2) && (filtre_baslama_1 == filtre_bitme_1))
  {
    if (hesaplanan_dakika < filtre_baslama_2 && hesaplanan_dakika >= filtre_bitme_2)
    {
      filtre_durum = 2;
    }
    else
    {
      filtre_durum = 3;
    }
  }
  else if (filtre_baslama_1 < filtre_bitme_1 && filtre_baslama_2 < filtre_bitme_2) // BU TAMAM
  {
    if (((filtre_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < filtre_bitme_1)) || ((filtre_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < filtre_bitme_2)))
    {
      filtre_durum = 3;
    }
    else
    {
      filtre_durum = 2;
    }
  }
  else if (filtre_baslama_1 > filtre_bitme_1 && filtre_baslama_2 > filtre_bitme_2) // BU TAMAM
  {
    if ((hesaplanan_dakika < filtre_baslama_1 && hesaplanan_dakika >= filtre_bitme_1) && (hesaplanan_dakika < filtre_baslama_2 && hesaplanan_dakika >= filtre_bitme_2))
    {
      filtre_durum = 2;
    }
    else
    {
      filtre_durum = 3;
    }
  }
  else if (filtre_baslama_1 > filtre_bitme_1 && filtre_baslama_2 < filtre_bitme_2)
  {
    if (((hesaplanan_dakika < filtre_baslama_1 && hesaplanan_dakika >= filtre_bitme_1) &&
         ((hesaplanan_dakika > filtre_baslama_2 && hesaplanan_dakika >= filtre_bitme_2) ||
          (hesaplanan_dakika < filtre_baslama_2 && hesaplanan_dakika <= filtre_bitme_2))))
    {
      filtre_durum = 2;
    }
    else
    {
      filtre_durum = 3;
    }
  }
  else if (filtre_baslama_1 < filtre_bitme_1 && filtre_baslama_2 > filtre_bitme_2)
  {
    if (((hesaplanan_dakika < filtre_baslama_2 && hesaplanan_dakika >= filtre_bitme_2) &&
         ((hesaplanan_dakika > filtre_baslama_1 && hesaplanan_dakika >= filtre_bitme_1) ||
          (hesaplanan_dakika < filtre_baslama_1 && hesaplanan_dakika <= filtre_bitme_1))))
    {
      filtre_durum = 2;
    }
    else
    {
      filtre_durum = 3;
    }
  }
}

void isik_oto_kontrol() // Isik oto timer kontrol
{
  isik_baslama_1 = (isik_timer_1_sh * 60) + isik_timer_1_sm;
  isik_bitme_1 = (isik_timer_1_eh * 60) + isik_timer_1_em;
  isik_baslama_2 = (isik_timer_2_sh * 60) + isik_timer_2_sm;
  isik_bitme_2 = (isik_timer_2_eh * 60) + isik_timer_2_em;

  if ((isik_baslama_1 == isik_bitme_1) && (isik_baslama_2 == isik_bitme_2))
  {
    isik_durum = 2;
  }
  else if ((isik_baslama_1 < isik_bitme_1) && (isik_baslama_2 == isik_bitme_2))
  {
    if ((isik_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < isik_bitme_1))
    {
      isik_durum = 3;
    }
    else
    {
      isik_durum = 2;
    }
  }
  else if ((isik_baslama_1 > isik_bitme_1) && (isik_baslama_2 == isik_bitme_2))
  {
    if (hesaplanan_dakika < isik_baslama_1 && hesaplanan_dakika >= isik_bitme_1)
    {
      isik_durum = 2;
    }
    else
    {
      isik_durum = 3;
    }
  }
  else if ((isik_baslama_2 < isik_bitme_2) && (isik_baslama_1 == isik_bitme_1))
  {
    if ((isik_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < isik_bitme_2))
    {
      isik_durum = 3;
    }
    else
    {
      isik_durum = 2;
    }
  }
  else if ((isik_baslama_2 > isik_bitme_2) && (isik_baslama_1 == isik_bitme_1))
  {
    if (hesaplanan_dakika < isik_baslama_2 && hesaplanan_dakika >= isik_bitme_2)
    {
      isik_durum = 2;
    }
    else
    {
      isik_durum = 3;
    }
  }
  else if (isik_baslama_1 < isik_bitme_1 && isik_baslama_2 < isik_bitme_2) // BU TAMAM
  {
    if (((isik_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < isik_bitme_1)) || ((isik_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < isik_bitme_2)))
    {
      isik_durum = 3;
    }
    else
    {
      isik_durum = 2;
    }
  }
  else if (isik_baslama_1 > isik_bitme_1 && isik_baslama_2 > isik_bitme_2) // BU TAMAM
  {
    if ((hesaplanan_dakika < isik_baslama_1 && hesaplanan_dakika >= isik_bitme_1) && (hesaplanan_dakika < isik_baslama_2 && hesaplanan_dakika >= isik_bitme_2))
    {
      isik_durum = 2;
    }
    else
    {
      isik_durum = 3;
    }
  }
  else if (isik_baslama_1 > isik_bitme_1 && isik_baslama_2 < isik_bitme_2)
  {
    if (((hesaplanan_dakika < isik_baslama_1 && hesaplanan_dakika >= isik_bitme_1) &&
         ((hesaplanan_dakika > isik_baslama_2 && hesaplanan_dakika >= isik_bitme_2) ||
          (hesaplanan_dakika < isik_baslama_2 && hesaplanan_dakika < isik_bitme_2))))
    {
      isik_durum = 2;
    }
    else
    {
      isik_durum = 3;
    }
  }
  else if (isik_baslama_1 < isik_bitme_1 && isik_baslama_2 > isik_bitme_2)
  {
    if (((hesaplanan_dakika < isik_baslama_2 && hesaplanan_dakika >= isik_bitme_2) &&
         ((hesaplanan_dakika > isik_baslama_1 && hesaplanan_dakika >= isik_bitme_1) ||
          (hesaplanan_dakika < isik_baslama_1 && hesaplanan_dakika <= isik_bitme_1))))
    {
      isik_durum = 2;
    }
    else
    {
      isik_durum = 3;
    }
  }
}

void aux1_oto_kontrol() // Aux-1 oto timer kontrol
{
  aux1_baslama_1 = (aux1_timer_1_sh * 60) + aux1_timer_1_sm;
  aux1_bitme_1 = (aux1_timer_1_eh * 60) + aux1_timer_1_em;
  aux1_baslama_2 = (aux1_timer_2_sh * 60) + aux1_timer_2_sm;
  aux1_bitme_2 = (aux1_timer_2_eh * 60) + aux1_timer_2_em;

  if ((aux1_baslama_1 == aux1_bitme_1) && (aux1_baslama_2 == aux1_bitme_2))
  {
    aux1_durum = 2;
  }
  else if ((aux1_baslama_1 < aux1_bitme_1) && (aux1_baslama_2 == aux1_bitme_2))
  {
    if ((aux1_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < aux1_bitme_1))
    {
      aux1_durum = 3;
    }
    else
    {
      aux1_durum = 2;
    }
  }
  else if ((aux1_baslama_1 > aux1_bitme_1) && (aux1_baslama_2 == aux1_bitme_2))
  {
    if (hesaplanan_dakika < aux1_baslama_1 && hesaplanan_dakika >= aux1_bitme_1)
    {
      aux1_durum = 2;
    }
    else
    {
      aux1_durum = 3;
    }
  }
  else if ((aux1_baslama_2 < aux1_bitme_2) && (aux1_baslama_1 == aux1_bitme_1))
  {
    if ((aux1_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < aux1_bitme_2))
    {
      aux1_durum = 3;
    }
    else
    {
      aux1_durum = 2;
    }
  }
  else if ((aux1_baslama_2 > aux1_bitme_2) && (aux1_baslama_1 == aux1_bitme_1))
  {
    if (hesaplanan_dakika < aux1_baslama_2 && hesaplanan_dakika >= aux1_bitme_2)
    {
      aux1_durum = 2;
    }
    else
    {
      aux1_durum = 3;
    }
  }
  else if (aux1_baslama_1 < aux1_bitme_1 && aux1_baslama_2 < aux1_bitme_2) // BU TAMAM
  {
    if (((aux1_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < aux1_bitme_1)) || ((aux1_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < aux1_bitme_2)))
    {
      aux1_durum = 3;
    }
    else
    {
      aux1_durum = 2;
    }
  }
  else if (aux1_baslama_1 > aux1_bitme_1 && aux1_baslama_2 > aux1_bitme_2) // BU TAMAM
  {
    if ((hesaplanan_dakika < aux1_baslama_1 && hesaplanan_dakika >= aux1_bitme_1) && (hesaplanan_dakika < aux1_baslama_2 && hesaplanan_dakika >= aux1_bitme_2))
    {
      aux1_durum = 2;
    }
    else
    {
      aux1_durum = 3;
    }
  }
  else if (aux1_baslama_1 > aux1_bitme_1 && aux1_baslama_2 < aux1_bitme_2)
  {
    if (((hesaplanan_dakika < aux1_baslama_1 && hesaplanan_dakika >= aux1_bitme_1) &&
         ((hesaplanan_dakika > aux1_baslama_2 && hesaplanan_dakika >= aux1_bitme_2) ||
          (hesaplanan_dakika < aux1_baslama_2 && hesaplanan_dakika <= aux1_bitme_2))))
    {
      aux1_durum = 2;
    }
    else
    {
      aux1_durum = 3;
    }
  }
  else if (aux1_baslama_1 < aux1_bitme_1 && aux1_baslama_2 > aux1_bitme_2)
  {
    if (((hesaplanan_dakika < aux1_baslama_2 && hesaplanan_dakika >= aux1_bitme_2) &&
         ((hesaplanan_dakika > aux1_baslama_1 && hesaplanan_dakika >= aux1_bitme_1) ||
          (hesaplanan_dakika < aux1_baslama_1 && hesaplanan_dakika <= aux1_bitme_1))))
    {
      aux1_durum = 2;
    }
    else
    {
      aux1_durum = 3;
    }
  }
}

void aux2_oto_kontrol() // Aux-2 oto timer kontrol
{
  aux2_baslama_1 = (aux2_timer_1_sh * 60) + aux2_timer_1_sm;
  aux2_bitme_1 = (aux2_timer_1_eh * 60) + aux2_timer_1_em;
  aux2_baslama_2 = (aux2_timer_2_sh * 60) + aux2_timer_2_sm;
  aux2_bitme_2 = (aux2_timer_2_eh * 60) + aux2_timer_2_em;

  if ((aux2_baslama_1 == aux2_bitme_1) && (aux2_baslama_2 == aux2_bitme_2))
  {
    aux2_durum = 2;
  }
  else if ((aux2_baslama_1 < aux2_bitme_1) && (aux2_baslama_2 == aux2_bitme_2))
  {
    if ((aux2_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < aux2_bitme_1))
    {
      aux2_durum = 3;
    }
    else
    {
      aux2_durum = 2;
    }
  }
  else if ((aux2_baslama_1 > aux2_bitme_1) && (aux2_baslama_2 == aux2_bitme_2))
  {
    if (hesaplanan_dakika < aux2_baslama_1 && hesaplanan_dakika >= aux2_bitme_1)
    {
      aux2_durum = 2;
    }
    else
    {
      aux2_durum = 3;
    }
  }
  else if ((aux2_baslama_2 < aux2_bitme_2) && (aux2_baslama_1 == aux2_bitme_1))
  {
    if ((aux2_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < aux2_bitme_2))
    {
      aux2_durum = 3;
    }
    else
    {
      aux2_durum = 2;
    }
  }
  else if ((aux2_baslama_2 > aux2_bitme_2) && (aux2_baslama_1 == aux2_bitme_1))
  {
    if (hesaplanan_dakika < aux2_baslama_2 && hesaplanan_dakika >= aux2_bitme_2)
    {
      aux2_durum = 2;
    }
    else
    {
      aux2_durum = 3;
    }
  }
  else if (aux2_baslama_1 < aux2_bitme_1 && aux2_baslama_2 < aux2_bitme_2) // BU TAMAM
  {
    if (((aux2_baslama_1 <= hesaplanan_dakika) && (hesaplanan_dakika < aux2_bitme_1)) || ((aux2_baslama_2 <= hesaplanan_dakika) && (hesaplanan_dakika < aux2_bitme_2)))
    {
      aux2_durum = 3;
    }
    else
    {
      aux2_durum = 2;
    }
  }
  else if (aux2_baslama_1 > aux2_bitme_1 && aux2_baslama_2 > aux2_bitme_2) // BU TAMAM
  {
    if ((hesaplanan_dakika < aux2_baslama_1 && hesaplanan_dakika >= aux2_bitme_1) && (hesaplanan_dakika < aux2_baslama_2 && hesaplanan_dakika >= aux2_bitme_2))
    {
      aux2_durum = 2;
    }
    else
    {
      aux2_durum = 3;
    }
  }
  else if (aux2_baslama_1 > aux2_bitme_1 && aux2_baslama_2 < aux2_bitme_2)
  {
    if (((hesaplanan_dakika < aux2_baslama_1 && hesaplanan_dakika >= aux2_bitme_1) &&
         ((hesaplanan_dakika > aux2_baslama_2 && hesaplanan_dakika >= aux2_bitme_2) ||
          (hesaplanan_dakika < aux2_baslama_2 && hesaplanan_dakika <= aux2_bitme_2))))
    {
      aux2_durum = 2;
    }
    else
    {
      aux2_durum = 3;
    }
  }
  else if (aux2_baslama_1 < aux2_bitme_1 && aux2_baslama_2 > aux2_bitme_2)
  {
    if (((hesaplanan_dakika < aux2_baslama_2 && hesaplanan_dakika >= aux2_bitme_2) &&
         ((hesaplanan_dakika > aux2_baslama_1 && hesaplanan_dakika >= aux2_bitme_1) ||
          (hesaplanan_dakika < aux2_baslama_1 && hesaplanan_dakika <= aux2_bitme_1))))
    {
      aux2_durum = 2;
    }
    else
    {
      aux2_durum = 3;
    }
  }
}

void eprom_update(int ep)
{
  if (ep == 1)
  {
    EEPROM.update(14, filtre_baslama_1);
    EEPROM.update(16, filtre_bitme_1);
    EEPROM.update(18, filtre_baslama_2);
    EEPROM.update(20, filtre_bitme_2);
  }
  else if (ep == 2)
  {
    EEPROM.update(22, isik_baslama_1);
    EEPROM.update(24, isik_bitme_1);
    EEPROM.update(26, isik_baslama_2);
    EEPROM.update(28, isik_bitme_2);
  }
  else if (ep == 3)
  {
    EEPROM.update(30, aux1_baslama_1);
    EEPROM.update(32, aux1_bitme_1);
    EEPROM.update(34, aux1_baslama_2);
    EEPROM.update(36, aux1_bitme_2);
  }
  else if (ep == 4)
  {
    EEPROM.update(38, aux2_baslama_1);
    EEPROM.update(40, aux2_bitme_1);
    EEPROM.update(42, aux2_baslama_2);
    EEPROM.update(44, aux2_bitme_2);
  }
}

void filtre_timer_1_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x10;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x11;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x12;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x13;
  Serial1.write(ekran_okuma, 7);
}

void filtre_timer_2_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x14;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x15;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x16;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x17;
  Serial1.write(ekran_okuma, 7);
}

void isik_timer_1_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x18;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x19;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x20;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x21;
  Serial1.write(ekran_okuma, 7);
}

void isik_timer_2_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x22;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x23;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x24;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x25;
  Serial1.write(ekran_okuma, 7);
}

void aux1_timer_1_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x26;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x27;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x28;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x29;
  Serial1.write(ekran_okuma, 7);
}

void aux1_timer_2_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x30;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x31;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x32;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x33;
  Serial1.write(ekran_okuma, 7);
}

void aux2_timer_1_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x34;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x35;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x36;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x37;
  Serial1.write(ekran_okuma, 7);
}

void aux2_timer_2_cek()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x38;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x39;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x40;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x41;
  Serial1.write(ekran_okuma, 7);
}

void saat_tarih_oku()
{
  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x52;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x53;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x54;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x55;
  Serial1.write(ekran_okuma, 7);

  ekran_okuma[4] = 0x22;
  ekran_okuma[5] = 0x56;
  Serial1.write(ekran_okuma, 7);
  zaman_cek = 0;
}

void ekran_saat_ayari_guncelle()
{
  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x52;
  komut_olustur(date.day + 600);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x53;
  komut_olustur(date.month + 600);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x54;
  komut_olustur(date.year - 2000 + 600);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x55;
  komut_olustur(time.hours + 600);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x56;
  komut_olustur(time.minutes + 600);
  Serial1.write(ekrana_yolla, 8);
}

void ekran_saat_set()
{
  if (saat_debug == 1)
  {
    Serial.println("=============================");
    Serial.print("Ekran Saat Ayari: ");
    Serial.print(time.hours);
    Serial.print(" : ");
    Serial.println(time.minutes);
    Serial.println("=============================");
  }
  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x50;
  komut_olustur(time.hours + 400);
  Serial1.write(ekrana_yolla, 8);
  delay(500);
  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x51;
  komut_olustur(time.minutes + 500);
  Serial1.write(ekrana_yolla, 8);
  time_save.minutes = time.minutes;
  saat_guncelle = 0;
}

void sistem_saat_ayarla()
{
  date.year = ekran_yil + 2000;
  date.month = ekran_ay;
  date.day = ekran_gun;
  time.hours = ekran_saat;
  time.minutes = ekran_dakika;
  time.seconds = 0;
  if (saat_debug == 1)
  {
    Serial.println("=============================");
    Serial.print("Ekrandan Gelen Saat Ayari: ");
    Serial.print(time.hours);
    Serial.print(" : ");
    Serial.print(time.minutes);
    Serial.print(" - ");
    Serial.print(date.day);
    Serial.print(" / ");
    Serial.print(date.month);
    Serial.print(" / ");
    Serial.println(date.year);
    Serial.println("=============================");
  }
  epochTime = rtc.dateTimeToEpoch(date, time);
  rtc.setTime(epochTime);
  saat_guncelle = 1;
  time_save.hours = time.hours;
}

void flow_alarm_ac()
{
  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x03;
  komut_olustur(75);
  Serial1.write(ekrana_yolla, 8);
  flow_alarm_eski = 0;
  flow_alarm = 0;
}

void backwash_menu()
{
  backwash_mod = 1;
  akim_pwm = 0;
  digitalWrite(role_3, LOW);
  guc_acik = 0;
  akim_pwm = 0;
  digitalWrite(filtre_pompa, LOW);
  filtre_durum = 0;
}

void backwash_ac()
{
  digitalWrite(filtre_pompa, HIGH);
  backwash_durum = 1;
  filtre_durum = 1;
}

void backwash_kapa()
{
  digitalWrite(filtre_pompa, LOW);
  backwash_durum = 0;
  filtre_durum = 0;
}

void backwash_cikis()
{
  backwash_mod = 0;
  backwash_durum = 0;
  filtre_durum = 0;
}

void flow_alarm_guncelle()
{
  if (flow_alarm != flow_alarm_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x03;
    if (flow_alarm == 0)
    {
      komut_olustur(75);
    }
    else if (flow_alarm == 1)
    {
      komut_olustur(76);
    }
    else if (flow_alarm == 2)
    {
      komut_olustur(77);
    }

    flow_alarm_eski = flow_alarm;

    Serial1.write(ekrana_yolla, 8);
  }
}

void backwash_alarm_guncelle(int alarm)
{
  if (alarm != backwash_alarm_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x04;
    komut_olustur(78 + alarm);
    if (alarm == 1)
    {
      backwash_alarm_eski = 1;
    }
    else if (alarm == 0)
    {
      backwash_alarm_eski = 0;
    }
    Serial1.write(ekrana_yolla, 8);
  }
}

void backwash_max_ekrana_gonder(int tim)
{
  memcpy(endian, &tim, 3);
  unsigned char calisma[8] = {0x5A, 0xA5, 0x05, 0x82, 0x22, 0x71, (unsigned char)endian[1], (unsigned char)endian[0]};
  Serial1.write(calisma, 8);
}

void backwash_ekrana_gonder(int tim)
{
  if (backwash_median_resim > 2333)
  {
    backwash_median_resim = 2272;
  }

  memcpy(endian, &tim, 3);

  switch (backwash_median_resim)
  {
  case 2272:
    backwash[4] = 0x22;
    backwash[5] = 0x72;
    break;
  case 2273:
    backwash[4] = 0x22;
    backwash[5] = 0x73;
    break;
  case 2274:
    backwash[4] = 0x22;
    backwash[5] = 0x74;
    break;
  case 2275:
    backwash[4] = 0x22;
    backwash[5] = 0x75;
    break;
  case 2276:
    backwash[4] = 0x22;
    backwash[5] = 0x76;
    break;
  case 2277:
    backwash[4] = 0x22;
    backwash[5] = 0x77;
    break;
  case 2278:
    backwash[4] = 0x22;
    backwash[5] = 0x78;
    break;
  case 2279:
    backwash[4] = 0x22;
    backwash[5] = 0x79;
    break;
  case 2280:
    backwash[4] = 0x22;
    backwash[5] = 0x80;
    break;
  case 2281:
    backwash[4] = 0x22;
    backwash[5] = 0x81;
    break;
  case 2282:
    backwash[4] = 0x22;
    backwash[5] = 0x82;
    break;
  case 2283:
    backwash[4] = 0x22;
    backwash[5] = 0x83;
    break;
  case 2284:
    backwash[4] = 0x22;
    backwash[5] = 0x84;
    break;
  case 2285:
    backwash[4] = 0x22;
    backwash[5] = 0x85;
    break;
  case 2286:
    backwash[4] = 0x22;
    backwash[5] = 0x86;
    break;
  case 2287:
    backwash[4] = 0x22;
    backwash[5] = 0x87;
    break;
  case 2288:
    backwash[4] = 0x22;
    backwash[5] = 0x88;
    break;
  case 2289:
    backwash[4] = 0x22;
    backwash[5] = 0x89;
    break;
  case 2290:
    backwash[4] = 0x22;
    backwash[5] = 0x90;
    break;
  case 2291:
    backwash[4] = 0x22;
    backwash[5] = 0x91;
    break;
  case 2292:
    backwash[4] = 0x22;
    backwash[5] = 0x92;
    break;
  case 2293:
    backwash[4] = 0x22;
    backwash[5] = 0x93;
    break;
  case 2294:
    backwash[4] = 0x22;
    backwash[5] = 0x94;
    break;
  case 2295:
    backwash[4] = 0x22;
    backwash[5] = 0x95;
    break;
  case 2296:
    backwash[4] = 0x22;
    backwash[5] = 0x96;
    break;
  case 2297:
    backwash[4] = 0x22;
    backwash[5] = 0x97;
    break;
  case 2298:
    backwash[4] = 0x22;
    backwash[5] = 0x98;
    break;
  case 2299:
    backwash[4] = 0x22;
    backwash[5] = 0x99;
    break;
  case 2300:
    backwash[4] = 0x23;
    backwash[5] = 0x00;
    break;
  case 2301:
    backwash[4] = 0x23;
    backwash[5] = 0x01;
    break;
  case 2302:
    backwash[4] = 0x23;
    backwash[5] = 0x02;
    break;
  case 2303:
    backwash[4] = 0x23;
    backwash[5] = 0x03;
    break;
  case 2304:
    backwash[4] = 0x23;
    backwash[5] = 0x04;
    break;
  case 2305:
    backwash[4] = 0x23;
    backwash[5] = 0x05;
    break;
  case 2306:
    backwash[4] = 0x23;
    backwash[5] = 0x06;
    break;
  case 2307:
    backwash[4] = 0x23;
    backwash[5] = 0x07;
    break;
  case 2308:
    backwash[4] = 0x23;
    backwash[5] = 0x08;
    break;
  case 2309:
    backwash[4] = 0x23;
    backwash[5] = 0x09;
    break;
  case 2310:
    backwash[4] = 0x23;
    backwash[5] = 0x10;
    break;
  case 2311:
    backwash[4] = 0x23;
    backwash[5] = 0x11;
    break;
  case 2312:
    backwash[4] = 0x23;
    backwash[5] = 0x12;
    break;
  case 2313:
    backwash[4] = 0x23;
    backwash[5] = 0x13;
    break;
  case 2314:
    backwash[4] = 0x23;
    backwash[5] = 0x14;
    break;
  case 2315:
    backwash[4] = 0x23;
    backwash[5] = 0x15;
    break;
  case 2316:
    backwash[4] = 0x23;
    backwash[5] = 0x16;
    break;
  case 2317:
    backwash[4] = 0x23;
    backwash[5] = 0x17;
    break;
  case 2318:
    backwash[4] = 0x23;
    backwash[5] = 0x18;
    break;
  case 2319:
    backwash[4] = 0x23;
    backwash[5] = 0x19;
    break;
  case 2320:
    backwash[4] = 0x23;
    backwash[5] = 0x20;
    break;
  case 2321:
    backwash[4] = 0x23;
    backwash[5] = 0x21;
    break;
  case 2322:
    backwash[4] = 0x23;
    backwash[5] = 0x22;
    break;
  case 2323:
    backwash[4] = 0x23;
    backwash[5] = 0x23;
    break;
  case 2324:
    backwash[4] = 0x23;
    backwash[5] = 0x24;
    break;
  case 2325:
    backwash[4] = 0x23;
    backwash[5] = 0x25;
    break;
  case 2326:
    backwash[4] = 0x23;
    backwash[5] = 0x26;
    break;
  case 2327:
    backwash[4] = 0x23;
    backwash[5] = 0x27;
    break;
  case 2328:
    backwash[4] = 0x23;
    backwash[5] = 0x28;
    break;
  case 2329:
    backwash[4] = 0x23;
    backwash[5] = 0x29;
    break;
  case 2330:
    backwash[4] = 0x23;
    backwash[5] = 0x30;
    break;
  case 2331:
    backwash[4] = 0x23;
    backwash[5] = 0x31;
    break;
  case 2332:
    backwash[4] = 0x23;
    backwash[5] = 0x32;
    break;
  case 2333:
    backwash[4] = 0x23;
    backwash[5] = 0x33;
    break;
  default:
    break;
  }
  backwash[6] = (unsigned char)endian[1];
  backwash[7] = (unsigned char)endian[0];

  Serial1.write(backwash, 8);
  backwash_median_resim++;
}

void ekran_komut_isle(int address1, int address2, int data1, int data2)
{
  if (adres_1 == 25)
  {
    if (adres_2 == 3 || adres_2 == 7) // Filtre Durum Sorgu
    {
      saat_gonder = 0;
      if (filtre_durum == 2 || filtre_durum == 3)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 5;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 36;
          Serial1.write(ekran_sayfa, 11);
        }
      }
      else
      {
        if ((filtre_durum == 1 || filtre_durum == 3))
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 4;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 35;
            Serial1.write(ekran_sayfa, 11);
          }
        }
        else if ((filtre_durum == 0 || filtre_durum == 2))
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 3;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 34;
            Serial1.write(ekran_sayfa, 11);
          }
        }
      }
    }
    else if (adres_2 == 16) // Filtre Manual On Geldi
    {
      filtre_durum = 1;
      eprom_donustur();
    }
    else if (adres_2 == 9) // Filtre Manual Off Geldi
    {
      filtre_durum = 0;
      eprom_donustur();
    }
    else if (adres_2 == 17) // Filtre Auto Geldi
    {
      filtre_durum = 2;
      eprom_donustur();
      filtre_oto_kontrol();
      eprom_update(1);
    }
    else if (adres_2 == 4 || adres_2 == 8) // Isik Durum Sorgusu
    {
      saat_gonder = 0;
      if (isik_durum == 2 || isik_durum == 3)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 12;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 43;
          Serial1.write(ekran_sayfa, 11);
        }
      }
      else if (isik_durum == 0 || isik_durum == 1)
      {
        if (isik_durum == 1 || isik_durum == 3)
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 11;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 42;
            Serial1.write(ekran_sayfa, 11);
          }
        }
        else if (isik_durum == 0 || isik_durum == 2)
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 10;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 41;
            Serial1.write(ekran_sayfa, 11);
          }
        }
      }
    }
    else if (adres_2 == 22) // Isik Manual Of geldi
    {
      isik_durum = 0;
      eprom_donustur();
    }
    else if (adres_2 == 23) // Isik Manual Oto Geldi
    {
      isik_durum = 1;
      eprom_donustur();
    }
    else if (adres_2 == 24) // Isik Oto Geldi
    {
      isik_durum = 2;
      eprom_donustur();
      isik_oto_kontrol();
      eprom_update(2);
    }
    else if (adres_2 == 5 || adres_2 == 25) // Aux-1 Durum Sorgu
    {
      saat_gonder = 0;
      if (aux1_durum == 2 || aux1_durum == 3)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 18;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 49;
          Serial1.write(ekran_sayfa, 11);
        }
      }
      else if (aux1_durum == 0 || aux1_durum == 1)
      {
        if (aux1_durum == 1 || aux1_durum == 3)
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 17;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 48;
            Serial1.write(ekran_sayfa, 11);
          }
        }
        else if (aux1_durum == 0 || aux1_durum == 2)
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 16;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 47;
            Serial1.write(ekran_sayfa, 11);
          }
        }
      }
    }
    else if (adres_2 == 33) // Aux-1 Manual Of Geldi
    {
      aux1_durum = 0;
      eprom_donustur();
    }
    else if (adres_2 == 34) // Aux-1 Manual On Geldi
    {
      aux1_durum = 1;
      eprom_donustur();
    }
    else if (adres_2 == 35) // Aux-1 Oto Geldi
    {
      aux1_durum = 2;
      eprom_donustur();
      aux1_oto_kontrol();
      eprom_update(3);
    }
    else if (adres_2 == 6 || adres_2 == 32) // Aux-2 Durum Sorgu
    {
      saat_gonder = 0;
      if (aux2_durum == 2 || aux2_durum == 3)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 23;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 54;
          Serial1.write(ekran_sayfa, 11);
        }
      }
      else if (aux2_durum == 0 || aux2_durum == 1)
      {
        if (aux2_durum == 1 || aux2_durum == 3)
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 22;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 53;
            Serial1.write(ekran_sayfa, 11);
          }
        }
        else if (aux2_durum == 0 || aux2_durum == 2)
        {
          if (dil == 0)
          {
            ekran_sayfa[9] = 21;
            Serial1.write(ekran_sayfa, 11);
          }
          else if (dil == 1)
          {
            ekran_sayfa[9] = 52;
            Serial1.write(ekran_sayfa, 11);
          }
        }
      }
    }
    else if (adres_2 == 36) // Aux-2 Manual Of Geldi
    {
      aux2_durum = 0;
      eprom_donustur();
    }
    else if (adres_2 == 37) // Aux-2 Manual On Geldi
    {
      aux2_durum = 1;
      eprom_donustur();
    }
    else if (adres_2 == 38) // Aux-2 Oto Geldi
    {
      aux2_durum = 2;
      eprom_donustur();
      aux2_oto_kontrol();
      eprom_update(4);
    }
    else if (adres_2 == 0) // Ana Ekran Low Alarm
    {
      if (low_alarm == 1)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 65;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 66;
          Serial1.write(ekran_sayfa, 11);
        }
      }
    }
    else if (adres_2 == 1) // Ana Ekran Flow Alarm
    {
      if (flow_alarm == 2 || flow_alarm == 0)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 63;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 64;
          Serial1.write(ekran_sayfa, 11);
        }
      }
    }
    else if (adres_2 == 2) // Ana Ekran Filt Alarm
    {
      if (backwash_alarm == 1)
      {
        if (dil == 0)
        {
          ekran_sayfa[9] = 67;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (dil == 1)
        {
          ekran_sayfa[9] = 69;
          Serial1.write(ekran_sayfa, 11);
        }
      }
    }
    else if (adres_2 == 52) // Saat tarih guncelleme geldi
    {
      saat_tarih_oku();
    }
    else if (adres_2 == 40)
    {
      dil = 0;
      eprom_donustur();
    }
    else if (adres_2 == 41)
    {
      dil = 1;
      eprom_donustur();
    }
    else if (adres_2 == 48)
    {
      akis_enable = 1;
      EEPROM.update(50, 1);
    }
    else if (adres_2 == 49)
    {
      akis_enable = 0;
      flow_alarm = 0;
      flow_alarm_guncelle();
      EEPROM.update(50, 0);

      // flow_alarm_ac();
    }
    else if (adres_2 == 64)
    {
      if (dil == 0)
      {
        ekran_sayfa[9] = 1;
        Serial1.write(ekran_sayfa, 11);
      }
      else if (dil == 1)
      {
        ekran_sayfa[9] = 32;
        Serial1.write(ekran_sayfa, 11);
      }
    }
    else if (adres_2 == 65)
    {
      if (dil == 0)
      {
        ekran_sayfa[9] = 28;
        Serial1.write(ekran_sayfa, 11);
      }
      else if (dil == 1)
      {
        ekran_sayfa[9] = 59;
        Serial1.write(ekran_sayfa, 11);
      }
      ekran_saat_ayari_guncelle();
    }
    else if (adres_2 == 69)
    {
      backwash_menu();
    }
    else if (adres_2 == 19)
    {
      backwash_cikis();
    }
    else if (adres_2 == 20)
    {
      backwash_kapa();
    }
    else if (adres_2 == 18)
    {
      backwash_ac();
    }
    else if (adres_2 == 70 || adres_2 == 71)
    {
      saat_gonder = 0;
    }
    else if (adres_2 == 72)
    {
      saat_gonder = 1;
      ekran_saat_set();
    }
    else if (adres_2 == 53)
    {
      backwash_max = 0;
      EEPROM.update(52, backwash_max);
      backwash_alarm_guncelle(0);
      backwash_max_ekrana_gonder(backwash_max);
    }
    else if (adres_2 == 80)
    {
      filtre_timer_1_cek();
    }
    else if (adres_2 == 81)
    {
      filtre_timer_2_cek();
    }
    else if (adres_2 == 82)
    {
      isik_timer_1_cek();
    }
    else if (adres_2 == 83)
    {
      isik_timer_2_cek();
    }
    else if (adres_2 == 84)
    {
      aux1_timer_1_cek();
    }
    else if (adres_2 == 85)
    {
      aux1_timer_2_cek();
    }
    else if (adres_2 == 86)
    {
      aux2_timer_1_cek();
    }
    else if (adres_2 == 87)
    {
      aux2_timer_2_cek();
    }
    else if (adres_2 == 88)
    {
      if (dil == 0)
      {
        ekran_sayfa[9] = 26;
        Serial1.write(ekran_sayfa, 11);
      }
      else if (dil == 1)
      {
        ekran_sayfa[9] = 57;
        Serial1.write(ekran_sayfa, 11);
      }
    }
    else if (adres_2 == 89)
    {
      if (dil == 0)
      {
        if (akis_enable == 0)
        {
          ekran_sayfa[9] = 74;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (akis_enable == 1)
        {
          ekran_sayfa[9] = 29;
          Serial1.write(ekran_sayfa, 11);
        }
      }
      else if (dil == 1)
      {
        if (akis_enable == 0)
        {
          ekran_sayfa[9] = 75;
          Serial1.write(ekran_sayfa, 11);
        }
        else if (akis_enable == 1)
        {
          ekran_sayfa[9] = 60;
          Serial1.write(ekran_sayfa, 11);
        }
      }
    }
  }
  else if (adres_1 == 34)
  {
    if (adres_2 == 69) // Uretim hedef geldi
    {
      uretim_hedef = data2 - 44;
      EEPROM.update(46, uretim_hedef);
      if (uretim_hedef == 0)
      {
        if (polarite == 1)
        {
          polarite_durum = 0;
        }
        else if (polarite == 2)
        {
          polarite_durum = 2;
        }
      }
    }
    else if (adres_2 == 82)
    {
      ekran_gun = data2 - 88;
    }
    else if (adres_2 == 83)
    {
      ekran_ay = data2 - 88;
    }
    else if (adres_2 == 84)
    {
      ekran_yil = data2 - 88;
    }
    else if (adres_2 == 85)
    {
      ekran_saat = data2 - 88;
    }
    else if (adres_2 == 86)
    {
      ekran_dakika = data2 - 88;
      sistem_saat_ayarla();
    }
    else if (adres_2 == 16)
    {
      filtre_timer_1_sh = data2 - 100;
    }
    else if (adres_2 == 17)
    {
      filtre_timer_1_sm = data2 - 100;
    }
    else if (adres_2 == 18)
    {
      filtre_timer_1_eh = data2 - 100;
    }
    else if (adres_2 == 19)
    {
      filtre_timer_1_em = data2 - 100;
    }
    else if (adres_2 == 20)
    {
      filtre_timer_2_sh = data2 - 100;
    }
    else if (adres_2 == 21)
    {
      filtre_timer_2_sm = data2 - 100;
    }
    else if (adres_2 == 22)
    {
      filtre_timer_2_eh = data2 - 100;
    }
    else if (adres_2 == 23)
    {
      filtre_timer_2_em = data2 - 100;
    }
    else if (adres_2 == 24)
    {
      isik_timer_1_sh = data2 - 100;
    }
    else if (adres_2 == 25)
    {
      isik_timer_1_sm = data2 - 100;
    }
    else if (adres_2 == 32)
    {
      isik_timer_1_eh = data2 - 100;
    }
    else if (adres_2 == 33)
    {
      isik_timer_1_em = data2 - 100;
    }
    else if (adres_2 == 34)
    {
      isik_timer_2_sh = data2 - 100;
    }
    else if (adres_2 == 35)
    {
      isik_timer_2_sm = data2 - 100;
    }
    else if (adres_2 == 36)
    {
      isik_timer_2_eh = data2 - 100;
    }
    else if (adres_2 == 37)
    {
      isik_timer_2_em = data2 - 100;
    }
    else if (adres_2 == 38)
    {
      aux1_timer_1_sh = data2 - 100;
    }
    else if (adres_2 == 39)
    {
      aux1_timer_1_sm = data2 - 100;
    }
    else if (adres_2 == 40)
    {
      aux1_timer_1_eh = data2 - 100;
    }
    else if (adres_2 == 41)
    {
      aux1_timer_1_em = data2 - 100;
    }
    else if (adres_2 == 48)
    {
      aux1_timer_2_sh = data2 - 100;
    }
    else if (adres_2 == 49)
    {
      aux1_timer_2_sm = data2 - 100;
    }
    else if (adres_2 == 50)
    {
      aux1_timer_2_eh = data2 - 100;
    }
    else if (adres_2 == 51)
    {
      aux1_timer_2_em = data2 - 100;
    }
    else if (adres_2 == 52)
    {
      aux2_timer_1_sh = data2 - 100;
    }
    else if (adres_2 == 53)
    {
      aux2_timer_1_sm = data2 - 100;
    }
    else if (adres_2 == 54)
    {
      aux2_timer_1_eh = data2 - 100;
    }
    else if (adres_2 == 55)
    {
      aux2_timer_1_em = data2 - 100;
    }
    else if (adres_2 == 56)
    {
      aux2_timer_2_sh = data2 - 100;
    }
    else if (adres_2 == 57)
    {
      aux2_timer_2_sm = data2 - 100;
    }
    else if (adres_2 == 64)
    {
      aux2_timer_2_eh = data2 - 100;
    }
    else if (adres_2 == 65)
    {
      aux2_timer_2_em = data2 - 100;
    }
  }
  else if (adres_1 == 35)
  {
    if (adres_2 == 52)
    {
      if (dil == 0)
      {
        ekran_sayfa[9] = 28;
        ekran_sayfa_degistir();
      }
      else if (dil == 1)
      {
        ekran_sayfa[9] = 59;
        ekran_sayfa_degistir();
      }
      ekran_saat_ayari_guncelle();
    }
    if (adres_2 == 53)
    {
      calisma_saati_guncelle(calisma_saat);
    }
  }
}

void low_alarm_guncelle(int alarm)
{
  if (alarm != low_alarm_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x01;
    komut_olustur(72 + alarm);
    if (alarm == 1)
    {
      low_alarm_eski = 1;
    }
    else if (alarm == 0)
    {
      low_alarm_eski = 0;
    }
    Serial1.write(ekrana_yolla, 8);
  }
}

void sensor_oku_baslangic()
{
  for (int i = 0; i < WINDOW_SIZE_A; i++)
  {
    akim_deger = analogRead(akim_sensor);
    READINGS_A[akim_index] = 0;
    READINGS_A[akim_index] = akim_deger;
    akim_index = (akim_index + 1) % WINDOW_SIZE_A;
  }

  sum_akim = 0;

  for (int i = 0; i < WINDOW_SIZE_A; i++)
  {
    sum_akim += READINGS_A[akim_index];
    akim_index = (akim_index + 1) % WINDOW_SIZE_A;
    ortalama_akim = sum_akim / WINDOW_SIZE_A;
  }

  akim_deger = ortalama_akim;
}

void sensor_oku()
{
  akim_anlik = analogRead(akim_sensor) + akim_anlik;
  akim_toplama_sayac++;
  if (akim_toplama_sayac > 500)
  {
    akim_deger = akim_anlik / akim_toplama_sayac;
    akim_anlik = 0;
    akim_toplama_sayac = 0;
  }
}

void serial_oku()
{
  if (Serial.available())
  {
    for (int i = 1; i < 6; i++)
    {
      cepGelen[i] = Serial.read();
    }
    if (cepGelen[2] < 11)
    {
      akim_pwm = (cepGelen[1] - 48) * 100;
    }
    if (cepGelen[3] < 11)
    {
      akim_pwm = (((cepGelen[1] - 48) * 10) + (cepGelen[2] - 48)) * 100;
    }
    if (cepGelen[4] < 11)
    {
      akim_pwm = (((cepGelen[1] - 48) * 100) + ((cepGelen[2] - 48) * 10) + (cepGelen[3] - 48)) * 100;
    }
    if (cepGelen[5] < 11)
    {
      akim_pwm = (((cepGelen[1] - 48) * 1000) + ((cepGelen[2] - 48) * 100) + ((cepGelen[3] - 48) * 10) + (cepGelen[4] - 48)) * 100;
    }

    Serial.begin(115200);
  }
}

void pwm_yaz()
{
  if (akim_deger < 1800)
  {
    akim_pwm = 5000;
  }
  else if (istenen_akim == 0)
  {
    akim_pwm = PWM_MINIMUM;
  }
  else if (akim_deger < istenen_akim)
  {
    if (akim_pwm < pwm_Max)
    {
      akim_pwm += 1;
    }
  }
  else if (akim_deger > istenen_akim)
  {
    akim_pwm = akim_pwm - 1;
  }

  if (akim_pwm > pwm_Max)
  {
    akim_pwm = pwm_Max;
  }

  if (akim_pwm < PWM_MINIMUM)
  {
    akim_pwm = PWM_MINIMUM;
  }
  pwmWrite(Pwm, akim_pwm);
}

void ekran_uretim_guncelle()
{
  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x00;
  if ((filtre_durum == 1 || filtre_durum == 3))
  {
    if (ekran_uretim > ekran_uretim_eski)
    {
      if (ekran_uretim_eski + 1 > kapasite)
      {
        komut_olustur(kapasite);
        Serial1.write(ekrana_yolla, 8);
      }
      else
      {
        komut_olustur(ekran_uretim_eski + 1);
        Serial1.write(ekrana_yolla, 8);
        ekran_uretim_eski += 1;
      }
    }
    else if (ekran_uretim < ekran_uretim_eski && ekran_uretim_eski != 0)
    {
      if (ekran_uretim_eski - 1 < 0)
      {
        komut_olustur(0);
        Serial1.write(ekrana_yolla, 8);
      }
      else
      {
        komut_olustur(ekran_uretim_eski - 1);
        Serial1.write(ekrana_yolla, 8);
        ekran_uretim_eski -= 1;
      }
    }
    // ekran_uretim_eski = ekran_uretim;
  }
  else if (filtre_durum == 0 || filtre_durum == 2)
  {
    komut_olustur(0);
    ekran_uretim_eski = 0;
    Serial1.write(ekrana_yolla, 8);
  }
}

void akis_kontrol()
{
  akis_deger = analogRead(akis_sensor);
  if (abs(akis_deger - akis_deger_onceki) > 200)
  {
    akis_hiz = 1;
  }
  else
  {
    akis_hiz = 0;
  }

  if (akis_hiz != akis_hiz_onceki)
  {
    akis_anlik = akis_anlik + 1;
  }
  akis_hiz_onceki = akis_hiz;

  if (millis() - akis_millis_sakla > 500)
  {
    if (akis_anlik == 0)
    {
      akim_pwm = PWM_MINIMUM;
      pwmWrite(Pwm, akim_pwm);
      flow_alarm = 2;
      flow_alarm_guncelle();
    }
    else
    {
      flow_alarm = 1;
      flow_alarm_guncelle();
    }
    backwash_saniye = akis_anlik;
    akis_anlik = 0;

    akis_millis_sakla = millis();
  }
}

void backwash_kontrol()
{
  backwash_deger = analogRead(akis_sensor);
  if (abs(backwash_deger - backwash_deger_onceki) > 200)
  {
    backwash_hiz = 1;
  }
  else
  {
    backwash_hiz = 0;
  }

  if (backwash_hiz != backwash_hiz_onceki)
  {
    backwash_anlik = backwash_anlik + 1;
  }
  backwash_hiz_onceki = backwash_hiz;
  if (millis() - backwash_millis > 32000)
  {
    backwash_millis = millis();
  }
  else if (millis() - backwash_millis > 30000)
  {
    backwash_rpm = backwash_anlik;
    backwash_anlik = 0;

    backwash_values[backwash_valueIndex] = backwash_rpm;
    backwash_valueIndex++;

    if (backwash_valueIndex == 5)
    {
      int sorted_backwash[5];
      for (int i = 0; i < 5; i++)
      {
        sorted_backwash[i] = backwash_values[i];
      }

      sort(sorted_backwash, sorted_backwash + 5);

      backwash_median = sorted_backwash[2];

      for (int i = 0; i < 4; i++)
      {
        backwash_values[i] = backwash_values[i + 1];
      }
      backwash_valueIndex = 4;
    }

    if (backwash_median > backwash_max)
    {
      backwash_max = backwash_median;
      EEPROM.update(52, backwash_max);
      backwash_max_ekrana_gonder(backwash_max);
    }
    backwash_ekrana_gonder(backwash_rpm);

    if (backwash_median >= backwash_max * 0.88)
    {
      backwash_alarm = 0;
      backwash_alarm_guncelle();
      backwash_counter = 0;
    }
    else if ((backwash_median != 0) && (backwash_median < backwash_max * 0.83))
    {
      backwash_counter++;
      if (backwash_counter > 1)
      {
        backwash_alarm = 1;
        backwash_alarm_guncelle();
        backwash_counter = 0;
      }
    }
    backwash_millis = millis();
  }
}

void hesaplamalar()
{

  if (filtre_durum == 1 || filtre_durum == 3)
  {

    if (sicaklik < sicaklik_2)
    {
      filtre_acik = 1;
    }
    else
    {
      filtre_acik = 0;
      if (filtre_durum_sicaklik == 0)
      {
        filtre_durum_sicaklik = filtre_durum;
        if (filtre_durum == 1)
        {
          filtre_durum = 0;
          low_alarm = 0;
        }
        else if (filtre_durum == 3)
        {
          filtre_durum = 2;
        }
      }
    }
  }
  else
  {
    if (sicaklik >= sicaklik_2)
    {
      filtre_acik = 0;
      low_alarm = 0;
    }
    else
    {
      if (filtre_durum_sicaklik != 0)
      {
        filtre_durum = filtre_durum_sicaklik;
        filtre_durum_sicaklik = 0;
      }
      else
      {
        filtre_acik = 0;
        low_alarm = 0;
      }
    }
  }

  if ((filtre_acik == 1) && (flow_alarm == 1 || akis_enable == 0) && backwash_mod == 0)
  {
    if (millis() - akim_millis_sakla > 4)
    {
      pwm_yaz();
      akim_millis_sakla = millis();
    }
  }

  if (filtre_acik == 1)
  {
    role_kontrol_set(filtre_pompa, 1);
  }
  else if (filtre_acik == 0)
  {
    role_kontrol_set(filtre_pompa, 0);
  }

  if (filtre_acik == 1 && akis_enable == 1 && backwash_mod == 0)
  {
    akis_kontrol();
  }

  // Uretim hedef algoritmasi
  if (uretim_hedef <= 0)
  {
    istenen_akim = 0;
    ekran_uretim = 0;
    role_kontrol_set(role_3, 0);
    if (polarite == 1)
    {
      polarite_durum = 0;
    }
    else if (polarite == 2)
    {
      polarite_durum = 2;
    }
    guc_acik = 0;
    akim_pwm = PWM_MINIMUM;
  }
  else if (uretim_hedef > kapasite)
  {
    uretim_hedef = kapasite;
  }
  else
  {
    istenen_akim = minimum_akim + (uretim_hedef * akim_katsayi);
  }

  // Ekrana gonderilecek uretim degeri
  if (akim_deger < minimum_akim || minimum_akim == 0)
  {
    ekran_uretim = 0;
  }
  else if (akim_deger > (minimum_akim + (akim_katsayi * kapasite)))
  {
    ekran_uretim = kapasite;
  }
  else
  {
    int fark = akim_deger - minimum_akim;
    float ekran_float = (float)(fark / akim_katsayi);
    ekran_uretim = round(ekran_float);
    if (ekran_uretim == uretim_hedef)
    {
      stab = millis();
    }
  }

  if (isik_durum == 1 || isik_durum == 3)
  {
    role_kontrol_set(havuz_isik, 1);
  }
  else if (isik_durum == 0 || isik_durum == 2)
  {
    role_kontrol_set(havuz_isik, 0);
  }

  if (aux1_durum == 1 || aux1_durum == 3)
  {
    role_kontrol_set(role_1, 1);
  }
  else if (aux1_durum == 0 || aux1_durum == 2)
  {
    role_kontrol_set(role_1, 0);
  }
  if (aux2_durum == 1 || aux2_durum == 3)
  {
    role_kontrol_set(role_2, 1);
  }
  else if (aux2_durum == 0 || aux2_durum == 2)
  {
    role_kontrol_set(role_2, 0);
  }

  if (akis_enable == 1 && backwash_mod == 0 && (filtre_durum == 1 || filtre_durum == 3) && flow_alarm == 1)
  {
    backwash_kontrol();
  }
}

void ekran_yaz()
{
  if (polarite_durum != polarite_durum_eski)
  {

    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x65;

    if (polarite_durum == 0)
    {
      komut_olustur(700);
    }
    else if (polarite_durum == 1)
    {
      komut_olustur(701);
    }
    else if (polarite_durum == 2)
    {
      komut_olustur(702);
    }
    else if (polarite_durum == 3)
    {
      komut_olustur(703);
    }
    Serial1.write(ekrana_yolla, 8);
    polarite_durum_eski = polarite_durum;
  }

  // Filtre Icon Guncelleme
  if (filtre_durum != filtre_durum_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x05;

    if (filtre_durum == 0)
    {
      komut_olustur(60);
      filtre_durum_eski = filtre_durum;
    }
    else if (filtre_durum == 1)
    {
      komut_olustur(61);
      filtre_durum_eski = filtre_durum;
    }
    else if (filtre_durum == 2)
    {
      komut_olustur(62);
      filtre_durum_eski = filtre_durum;
    }
    else if (filtre_durum == 3)
    {
      komut_olustur(63);
      filtre_durum_eski = filtre_durum;
    }
    Serial1.write(ekrana_yolla, 8);
  }

  // Isik Icon Guncelleme
  if (isik_durum != isik_durum_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x06;
    if (isik_durum == 0)
    {
      komut_olustur(64);
      isik_durum_eski = isik_durum;
    }
    else if (isik_durum == 1)
    {
      komut_olustur(65);
      isik_durum_eski = isik_durum;
    }
    else if (isik_durum == 2)
    {
      komut_olustur(66);
      isik_durum_eski = isik_durum;
    }
    else if (isik_durum == 3)
    {
      komut_olustur(67);
      isik_durum_eski = isik_durum;
    }
    Serial1.write(ekrana_yolla, 8);
  }

  // Aux-1 Icon Guncelleme
  if (aux1_durum != aux1_durum_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x08;
    if (aux1_durum == 0)
    {
      komut_olustur(68);
      aux1_durum_eski = aux1_durum;
    }
    else if (aux1_durum == 1)
    {
      komut_olustur(69);
      aux1_durum_eski = aux1_durum;
    }
    else if (aux1_durum == 2)
    {
      komut_olustur(70);
      aux1_durum_eski = aux1_durum;
    }
    else if (aux1_durum == 3)
    {
      komut_olustur(71);
      aux1_durum_eski = aux1_durum;
    }
    Serial1.write(ekrana_yolla, 8);
  }

  // Aux-2 Icon Guncelleme

  if (aux2_durum != aux2_durum_eski)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x09;
    if (aux2_durum == 0)
    {
      komut_olustur(68);
      aux2_durum_eski = aux2_durum;
    }
    else if (aux2_durum == 1)
    {
      komut_olustur(69);
      aux2_durum_eski = aux2_durum;
    }
    else if (aux2_durum == 2)
    {
      komut_olustur(70);
      aux2_durum_eski = aux2_durum;
    }
    else if (aux2_durum == 3)
    {
      komut_olustur(71);
      aux2_durum_eski = aux2_durum;
    }
    Serial1.write(ekrana_yolla, 8);
  }

  if (uretim_hedef_guncelle == 1)
  {
    ekrana_yolla[4] = 0x22;
    ekrana_yolla[5] = 0x45;
    komut_olustur(uretim_hedef + 300);
    Serial1.write(ekrana_yolla, 8);
    uretim_hedef_guncelle = 0;
  }
}

void ekran_oku()
{
  if (Serial1.available())
  {
    int ekran_byte = Serial1.read();

    if (ekran_byte == 0x5A)
    {
      byte_kontrol = 1;
      ekran_buffer = 0;
    }

    if (ekran_byte == 0xA5 && byte_kontrol == 1)
    {
      ekran_buffer = 1;
      byte_kontrol = 0;
    }

    if (ekran_buffer == 2)
    {
      byte_uzunluk = ekran_byte + 2;
      if (byte_uzunluk > 12)
      {
        byte_uzunluk = 12;
      }
    }

    if (byte_uzunluk < 11)
    {
      if (byte_kontrol == 0)
      {
        ekran_gelen[ekran_buffer] = ekran_byte;
      }

      if (ekran_buffer == byte_uzunluk)
      {
        adres_1 = ekran_gelen[4];
        adres_2 = ekran_gelen[5];
        data_1 = ekran_gelen[7];
        data_2 = ekran_gelen[8];
        ekran_komut_isle(adres_1, adres_2, data_1, data_2);
        ekran_buffer = 0;
        if (ekran_debug == 1)
        {
          for (int i = 0; i < byte_uzunluk; i++)
          {
            Serial.print(ekran_gelen[i], HEX);
            Serial.print(" ");
            ekran_gelen[i] = 0;
          }
          Serial.println();
          Serial.println("========================");
        }
      }
    }
    ekran_buffer++;
  }
}

void filtre_oto_zaman_guncelle()
{
  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x10;
  komut_olustur(filtre_timer_1_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x11;
  komut_olustur(filtre_timer_1_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x12;
  komut_olustur(filtre_timer_1_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x13;
  komut_olustur(filtre_timer_1_em + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x14;
  komut_olustur(filtre_timer_2_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x15;
  komut_olustur(filtre_timer_2_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x16;
  komut_olustur(filtre_timer_2_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x17;
  komut_olustur(filtre_timer_2_em + 100);
  Serial1.write(ekrana_yolla, 8);
  filtre_timer_guncelle = 0;
}

void isik_oto_zaman_guncelle()
{

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x18;
  komut_olustur(isik_timer_1_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x19;
  komut_olustur(isik_timer_1_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x20;
  komut_olustur(isik_timer_1_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x21;
  komut_olustur(isik_timer_1_em + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x22;
  komut_olustur(isik_timer_2_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x23;
  komut_olustur(isik_timer_2_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x24;
  komut_olustur(isik_timer_2_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x25;
  komut_olustur(isik_timer_2_em + 100);
  Serial1.write(ekrana_yolla, 8);
  isik_timer_guncelle = 0;
}

void aux1_oto_zaman_guncelle()
{

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x26;
  komut_olustur(aux1_timer_1_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x27;
  komut_olustur(aux1_timer_1_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x28;
  komut_olustur(aux1_timer_1_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x29;
  komut_olustur(aux1_timer_1_em + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x30;
  komut_olustur(aux1_timer_2_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x31;
  komut_olustur(aux1_timer_2_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x32;
  komut_olustur(aux1_timer_2_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x33;
  komut_olustur(aux1_timer_2_em + 100);
  Serial1.write(ekrana_yolla, 8);
  aux1_timer_guncelle = 0;
}

void aux2_oto_zaman_guncelle()
{

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x34;
  komut_olustur(aux2_timer_1_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x35;
  komut_olustur(aux2_timer_1_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x36;
  komut_olustur(aux2_timer_1_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x37;
  komut_olustur(aux2_timer_1_em + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x38;
  komut_olustur(aux2_timer_2_sh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x39;
  komut_olustur(aux2_timer_2_sm + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x40;
  komut_olustur(aux2_timer_2_eh + 100);
  Serial1.write(ekrana_yolla, 8);

  ekrana_yolla[4] = 0x22;
  ekrana_yolla[5] = 0x41;
  komut_olustur(aux2_timer_2_em + 100);
  Serial1.write(ekrana_yolla, 8);
  aux2_timer_guncelle = 0;
}

void polarite_degistir(int pol)
{
  if (pol != polarite)
  {
    if (pol == 1)
    {
      akim_pwm = 0;
      pwmWrite(Pwm, akim_pwm);
      delay(2000);
      digitalWrite(role_3, LOW);

      guc_acik = 0;
      akim_pwm = 0;
      polarite_durum = 0;
      delay(2000);
      digitalWrite(role_4, LOW);

      delay(2000);
      if (uretim_hedef > 0)
      {
        digitalWrite(role_3, HIGH);

        guc_acik = 1;
        polarite_durum = 1;
      }
      polarite = 2;
    }
    else if (pol == 2)
    {
      akim_pwm = 0;
      pwmWrite(Pwm, akim_pwm);
      delay(2000);
      digitalWrite(role_3, LOW);

      guc_acik = 0;
      akim_pwm = 0;
      polarite_durum = 2;
      delay(2000);
      digitalWrite(role_4, HIGH);

      delay(2000);
      if (uretim_hedef > 0)
      {
        digitalWrite(role_3, HIGH);

        guc_acik = 1;
        polarite_durum = 3;
      }
      polarite = 1;
    }
    polarite = pol;
  }
}

void her_250_calisan()
{
  if (millis() - millis_250 > 240)
  {
    if (ekran_uretim != ekran_uretim_eski)
    {
      if (millis() - stab > 750 || ekran_uretim == uretim_hedef)
      {
        ekran_uretim_guncelle();
      }
    }
    ekran_yaz();
    millis_250 = millis();
  }
}

void printTime()
{
  if (saat_debug == 1)
  {
    Serial.println("=============================");
    Serial.print(time.hours);
    Serial.print(" : ");
    Serial.print(time.minutes);
    Serial.print(" : ");
    Serial.println(time.seconds);
    Serial.println("=============================");
  }
}

void sicaklik_guncelle()
{
  if (sicaklik != sicaklik_eski && sicaklik < 100 && sicaklik > -5)
  {
    sicaklik_eski = sicaklik;
    ekrana_yolla[4] = 0x23;
    ekrana_yolla[5] = 0x36;
    if (sicaklik > 50 && sicaklik < 70)
    {
      komut_olustur(705);
    }
    else if (sicaklik > 70)
    {
      komut_olustur(704);
    }
    else
    {
      komut_olustur(703);
    }
    Serial1.write(ekrana_yolla, 8);
    int sicak = round(sicaklik);
    memcpy(endian4, &sicak, 3);
    unsigned char sic[8] = {0x5A, 0xA5, 0x05, 0x82, 0x23, 0x37, (unsigned char)endian4[1], (unsigned char)endian4[0]};
    Serial1.write(sic, 8);
  }
}

float sicaklik_olc(OneWire &ds, byte start)
{
  int16_t temp;
  do
  {
    ds.reset();
    ds.write(0XCC);
    ds.write(0XBE);
    ds.read_bytes((uint8_t *)&temp, sizeof(temp));
    ds.reset();
    ds.write(0xCC);
    ds.write(0x44, 1);
    if (start)
    {
      delay(500);
    }
  } while (start--);

  return (temp * 0.0625);
}

void her_saniye_calisan()
{
  if (millis() - millis_sakla > 1000)
  {
    if (genel_debug == 1)
    {
      Serial.println("FILTRASYON: " + String(filtre_durum) + " - ELEK ACIK: " + String(guc_acik) + " - ELEK SET: " + String(uretim_hedef) + " - URETIM: " + String(ekran_uretim) + " - PWM: " + String(akim_pwm) + " - AKIM SENS: " + String(akim_deger) + " - AKIM MIN: " + String(minimum_akim) + " - CALISMA: " + String(calisma_saat) + " - AKIS ALARM: " + String(flow_alarm) + " - POLARITE: " + String(polarite) + " - AKIS: " + String(akis_anlik) + " - BACKW MAX: " + String(backwash_max) + " - BACK RPM: " + String(backwash_rpm) + " - BACK ALARM: " + String(backwash_alarm) + " - DUSUK ALARM: " + String(low_alarm) + " - SICAKLIK: " + String(sicaklik));
    }

    if ((filtre_acik == 1) && backwash_mod == 0 && uretim_hedef > 0)
    {
      backwash_anlik_ekrana_gonder(backwash_saniye);
      if (akis_enable == 1)
      {
        if (flow_alarm == 1)
        {
          role_kontrol_set(role_3, 1);
          guc_acik = 1;
          calisma_sayac++;
          if (calisma_sayac >= 3600)
          {
            calisma_saat++;
            polarite_timer_hour++;
            calisma_sayac = 0;
          }
          polarite_set();
        }
        else if (flow_alarm == 2)
        {
          role_kontrol_set(role_3, 0);
          guc_acik = 0;
          akim_pwm = PWM_MINIMUM;
          if (polarite == 1)
          {
            polarite_durum = 0;
          }
          else if (polarite == 2)
          {
            polarite_durum = 2;
          }
        }
      }
      else if (akis_enable == 0)
      {
        role_kontrol_set(role_3, 1);
        guc_acik = 1;
        calisma_sayac++;
        if (calisma_sayac >= 3600)
        {
          calisma_saat++;
          polarite_timer_hour++;
          calisma_sayac = 0;
        }
        if (polarite == 1)
        {
          polarite_durum = 1;
        }
        else if (polarite == 2)
        {
          polarite_durum = 3;
        }
      }
    }
    else if (filtre_durum == 0 || filtre_durum == 2)
    {
      role_kontrol_set(role_3, 0);
      guc_acik = 0;
      backwash_anlik = 0;
      backwash_valueIndex = 0;
      if (polarite == 1)
      {
        polarite_durum = 0;
      }
      else if (polarite == 2)
      {
        polarite_durum = 2;
      }
      akim_pwm = PWM_MINIMUM;
      if (polarite == 1)
      {
        polarite_durum = 0;
      }
      else if (polarite == 2)
      {
        polarite_durum = 2;
      }

      if (akis_enable == 1)
      {
        flow_alarm = 1;
        flow_alarm_guncelle();
      }

      akim_pwm = PWM_MINIMUM;
      pwmWrite(Pwm, akim_pwm);
    }

    if (polarite_timer_hour >= 5)
    {
      if (((polarite_timer_hour / POLARITE_TIMER) % 2) == 0)
      {
        polarite_degistir(1);
        EEPROM.update(62, 1);
      }
      else if (((polarite_timer_hour / POLARITE_TIMER) % 2) == 1)
      {
        polarite_degistir(2);
        EEPROM.update(62, 2);
      }
    }

    if (zaman_cek == 1)
    {
      saat_tarih_oku();
    }

    if (filtre_timer_guncelle == 1)
    {
      filtre_oto_zaman_guncelle();
    }

    if (isik_timer_guncelle == 1)
    {
      isik_oto_zaman_guncelle();
    }

    if (aux1_timer_guncelle == 1)
    {
      aux1_oto_zaman_guncelle();
    }

    if (aux2_timer_guncelle == 1)
    {
      aux2_oto_zaman_guncelle();
    }

    if (akim_pwm > pwm_Max * 0.95)
    {
      low_alarm = 1;
      low_alarm_guncelle(low_alarm);
    }
    else if (akim_pwm < pwm_Max * 0.9)
    {
      low_alarm = 0;
      low_alarm_guncelle(low_alarm);
    }

    sicaklik = sicaklik_olc(oneWire, 0);

    sicaklik_guncelle();

    millis_sakla = millis();
  }
}

void her_dakika_calisan()
{
  if (time.minutes != time_save.minutes)
  {
    hesaplanan_dakika = time.hours * 60 + time.minutes;

    if (filtre_durum == 2 || filtre_durum == 3)
    {
      filtre_oto_kontrol();
    }

    if (isik_durum == 2 || isik_durum == 3)
    {
      isik_oto_kontrol();
    }

    if (aux1_durum == 2 || aux1_durum == 3)
    {
      aux1_oto_kontrol();
    }

    if (aux2_durum == 2 || aux2_durum == 3)
    {
      aux2_oto_kontrol();
    }

    if (saat_gonder == 1)
    {
      ekran_saat_set();
    }

    if ((time.hours == 10 || time.hours == 13 || time.hours == 16 || time.hours == 19) && time.minutes == 1)
    {
      EEPROM.update(48, calisma_saat);
    }

    time_save.minutes = time.minutes;
  }
  if (time.hours != time_save.hours)
  {
    time_save.hours = time.hours;
  }
}

void eprom_temizle()
{
  EEPROM.write(10, 0);
  EEPROM.write(14, 0);
  EEPROM.write(16, 0);
  EEPROM.write(18, 0);
  EEPROM.write(20, 0);
  EEPROM.write(22, 0);
  EEPROM.write(24, 0);
  EEPROM.write(26, 0);
  EEPROM.write(28, 0);
  EEPROM.write(30, 0);
  EEPROM.write(32, 0);
  EEPROM.write(34, 0);
  EEPROM.write(36, 0);
  EEPROM.write(38, 0);
  EEPROM.write(40, 0);
  EEPROM.write(42, 0);
  EEPROM.write(44, 0);
  EEPROM.write(46, 0);
  EEPROM.write(48, 0);
  EEPROM.write(50, 0);
  EEPROM.write(52, 0);
}

void eprom_oku()
{

  /* Epromdaki Filtrasyon Durum */
  eprom_deger = EEPROM.read(10);

  filtre_durum = eprom_deger / 10000;
  isik_durum = (eprom_deger / 1000) % 10;
  aux1_durum = (eprom_deger / 100) % 10;
  aux2_durum = (eprom_deger / 10) % 10;
  dil = eprom_deger % 10;

  filtre_t1_s_eprom = EEPROM.read(14);
  filtre_timer_1_sh = filtre_t1_s_eprom / 60;
  filtre_timer_1_sm = filtre_t1_s_eprom % 60;

  filtre_t1_e_eprom = EEPROM.read(16);

  filtre_timer_1_eh = filtre_t1_e_eprom / 60;
  filtre_timer_1_em = filtre_t1_e_eprom % 60;

  filtre_t2_s_eprom = EEPROM.read(18);

  filtre_timer_2_sh = filtre_t2_s_eprom / 60;
  filtre_timer_2_sm = filtre_t2_s_eprom % 60;

  filtre_t2_e_eprom = EEPROM.read(20);

  filtre_timer_2_eh = filtre_t2_e_eprom / 60;
  filtre_timer_2_em = filtre_t2_e_eprom % 60;

  isik_t1_s_eprom = EEPROM.read(22);

  isik_timer_1_sh = isik_t1_s_eprom / 60;
  isik_timer_1_sm = isik_t1_s_eprom % 60;

  isik_t1_e_eprom = EEPROM.read(24);

  isik_timer_1_eh = isik_t1_e_eprom / 60;
  isik_timer_1_em = isik_t1_e_eprom % 60;

  isik_t2_s_eprom = EEPROM.read(26);

  isik_timer_2_sh = isik_t2_s_eprom / 60;
  isik_timer_2_sm = isik_t2_s_eprom % 60;

  isik_t2_e_eprom = EEPROM.read(28);

  isik_timer_2_eh = isik_t2_e_eprom / 60;
  isik_timer_2_em = isik_t2_e_eprom % 60;

  aux1_t1_s_eprom = EEPROM.read(30);

  aux1_timer_1_sh = aux1_t1_s_eprom / 60;
  aux1_timer_1_sm = aux1_t1_s_eprom % 60;

  aux1_t1_e_eprom = EEPROM.read(32);

  aux1_timer_1_eh = aux1_t1_e_eprom / 60;
  aux1_timer_1_em = aux1_t1_e_eprom % 60;

  aux1_t2_s_eprom = EEPROM.read(34);

  aux1_timer_2_sh = aux1_t2_s_eprom / 60;
  aux1_timer_2_sm = aux1_t2_s_eprom % 60;

  aux1_t2_e_eprom = EEPROM.read(36);

  aux1_timer_2_eh = aux1_t2_e_eprom / 60;
  aux1_timer_2_em = aux1_t2_e_eprom % 60;

  aux2_t1_s_eprom = EEPROM.read(38);

  aux2_timer_1_sh = aux2_t1_s_eprom / 60;
  aux2_timer_1_sm = aux2_t1_s_eprom % 60;

  aux2_t1_e_eprom = EEPROM.read(40);

  aux2_timer_1_eh = aux2_t1_e_eprom / 60;
  aux2_timer_1_em = aux2_t1_e_eprom % 60;

  aux2_t2_s_eprom = EEPROM.read(42);

  aux2_timer_2_sh = aux2_t2_s_eprom / 60;
  aux2_timer_2_sm = aux2_t2_s_eprom % 60;

  aux2_t2_e_eprom = EEPROM.read(44);

  aux2_timer_2_eh = aux2_t2_e_eprom / 60;
  aux2_timer_2_em = aux2_t2_e_eprom % 60;

  uretim_hedef = EEPROM.read(46);

  calisma_eprom = EEPROM.read(48);
  calisma_saat = calisma_eprom;
  akis_enable = EEPROM.read(50);
  // EEPROM.update(52, 0);
  // delay(1000);
  backwash_max = EEPROM.read(52);
  backwash_max_ekrana_gonder(backwash_max);

  polarite = EEPROM.read(62);
  polariteEprom = polarite;

  if (polarite > 3 || polarite <= 0)
  {
    polarite = 1;
  }

  Serial.println("========== EPROM LOADED ==========");
}

void polarite_baslangic()
{
  if (polarite == 1)
  {
    pwmWrite(Pwm, PWM_MINIMUM);
    digitalWrite(role_3, LOW);

    akim_pwm = 0;
    guc_acik = 0;
    polarite_durum = 0;
    delay(2000);
    digitalWrite(role_4, LOW);

    delay(2000);
    if (uretim_hedef > 0)
    {
      digitalWrite(role_3, HIGH);

      guc_acik = 1;
      polarite_durum = 1;
    }
  }
  else if (polarite == 2)
  {
    pwmWrite(Pwm, PWM_MINIMUM);
    digitalWrite(role_3, LOW);

    guc_acik = 0;
    polarite_durum = 2;
    akim_pwm = 0;
    delay(2000);
    digitalWrite(role_4, HIGH);

    delay(2000);
    if (uretim_hedef > 0)
    {
      digitalWrite(role_3, HIGH);

      guc_acik = 1;
      polarite_durum = 3;
    }
  }
}

void setup()
{
  EEPROM.init();
  // put your setup code here, to run once:
  pwmWrite(Pwm, akim_pwm);
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial1.begin(115200); // DWIN Lcd portu. 5V. SWD3
  Serial1.setTimeout(1);
  Serial2.begin(1200);
  Serial3.begin(19200);
  Serial3.setTimeout(1);

  rtc.begin();
  isHour12 = IS_HOUR_12(rtc);
  epochTime = rtc.getTime();
  rtc.epochToDate(epochTime, date);
  rtc.epochToTime(epochTime, time);
  saat_guncelle = 1;

  pinMode(akim_sensor, INPUT);
  pinMode(akis_sensor, INPUT);
  pinMode(volt_sensor, INPUT);
  pinMode(DS18B20_sens, INPUT);
  pinMode(V5_sensor, INPUT);
  pinMode(filtre_pompa, OUTPUT);
  pinMode(havuz_isik, OUTPUT);
  pinMode(role_1, OUTPUT);
  pinMode(role_2, OUTPUT);
  pinMode(role_3, OUTPUT);
  pinMode(role_4, OUTPUT);
  pinMode(bilgi_led, OUTPUT);
  pinMode(Pwm, PWM);

  Timer4.pause();
  Timer4.setPrescaleFactor(1);
  Timer4.setOverflow(8200); // 8628khz  8200
  Timer4.refresh();
  Timer4.resume();

  digitalWrite(bilgi_led, HIGH);

  if (EEPROM.read(100) != 9)
  {
    eprom_temizle();
    EEPROM.update(100, 9);
  }

  if (calisma_sayac_sifirla == 1)
  {
    EEPROM.update(48, 0);
    calisma_sayac_sifirla = 0;
  }

  hesaplanan_dakika = time.hours * 60 + time.minutes;

  eprom_oku();
  // EEPROM.format();
  // delay(50);
  filtre_timer_guncelle = 1;
  isik_timer_guncelle = 1;
  aux1_timer_guncelle = 1;
  aux2_timer_guncelle = 1;
  uretim_hedef_guncelle = 1;
  ekran_saat_ayari_guncelle();

  if (akis_enable == 0)
  {
    flow_alarm = 0;
    flow_alarm_guncelle();
  }

  if (filtre_durum == 2 || filtre_durum == 3)
  {
    filtre_oto_kontrol();
  }
  if (isik_durum == 2 || isik_durum == 3)
  {
    isik_oto_kontrol();
  }
  if (aux1_durum == 2 || aux1_durum == 3)
  {
    aux1_oto_kontrol();
  }
  if (aux2_durum == 2 || aux2_durum == 3)
  {
    aux2_oto_kontrol();
  }

  delay(2000);
  if (dil == 0)
  {
    ekran_sayfa[9] = 1;
    ekran_sayfa_degistir();
  }
  else if (dil == 1)
  {
    ekran_sayfa[9] = 32;
    ekran_sayfa_degistir();
  }

  sicaklik_olc(oneWire, 1);

  update_version();

  polarite_baslangic();
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (rtc.isCounterUpdated())
  {
    rtc.clearSecondFlag();
    epochTime = rtc.getTime();
    rtc.epochToTime(epochTime, time);
    printTime();
  }

  if (saat_guncelle == 1)
  {
    ekran_saat_set();
  }
  her_250_calisan();
  her_saniye_calisan();
  her_dakika_calisan();
  hesaplamalar();
  sensor_oku();
  ekran_oku();
  serial_oku();
}
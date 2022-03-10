#include <Wire.h>
#include "mcp9808.h"
#define max_period 80 // bir periyottaki maksimum ornek sayısı
#define olcumler 10      // saklanan doğru ölçüm sayısı
#define orn_siz 4       // ortalama için alınan ornek sayısı
#define rise_threshold 3 // bir atım için hesaplanan olcumler
int T = 20;              
int sensorPin = A1;   //alıcı IR led pini
int REDLed = 3;       //kırmızı led pini
int IRLed = 4;        //kızılötesi led pini
int lastSPO2 = 0;
int lastortBPM = 0;
float lastvSicaklik = 0;
MCP9808 sicaklikSensor(24);
void setup() {
    Serial.begin(115200);
    pinMode(sensorPin,INPUT);   //IR alıcı, giriş olarak ayarlanıyor
    pinMode(REDLed,OUTPUT);     //kırmızı ve IR led çıkış olarak ayarlanıyor.
    pinMode(IRLed,OUTPUT);
    sicaklikSensor.setResolution(3);
}
void loop (){
      int spo2;
      int nabiz;
      bool parmakKontrol = true;
      float okuIR[orn_siz], ortIR,sonIR, reader, start;
      float okuRED[orn_siz], ortRED,sonRED;
      int period, ornekler;
      period=0; ornekler=0;
      int ornekSayaci = 0;
      float okuIRMM[max_period],okuREDMM[max_period];
      int ptrMM =0;
      for (int i = 0; i < max_period; i++) { okuIRMM[i] = 0;okuREDMM[i]=0;}
      float IRmax=0;
      float IRmin=0;
      float REDmax=0;
      float REDmin=0;
      double R=0;
      float olcumR[olcumler];
      int olcumPeriod[olcumler];
      int m = 0;
      for (int i = 0; i < olcumler; i++) { olcumPeriod[i]=0; olcumR[i]=0; }
      int ptr;
      float oncekiIR;
      bool yukselen;
      int yuks_say;
      int n;
      long int son_atim;
      for (int i = 0; i < orn_siz; i++) { okuIR[i] = 0; okuRED[i]=0; }
      ortIR = 0; ortRED=0; 
      ptr = 0; 
  while(1)
  {
    // kızılotesi led açılır
    digitalWrite(REDLed,LOW);
    digitalWrite(IRLed,HIGH);
    // 20ms'lik periyot boyunca sensorden alınan ortalama hesaplanır.
    // bu elektrikten kaynaklanan 50Hz gurultuyu elimine eder
    n = 0;
    start = millis();
    reader = 0.;
    do
    {
      reader += analogRead (sensorPin);
      n++;
    }while (millis() < start + T);
    reader /= n;  // ortalamamız var
    // En yeni olçum diziye atanır, en eski olçum duzuden çıkarılır.  
    // Boylece son olcumlerin ortalaması hesaplamaya doğru şekilde katılır
    ortIR -= okuIR[ptr];
    ortIR += reader;
    okuIR[ptr] = reader;
    sonIR = ortIR / orn_siz;
    // kırmızı led açılır ve aynısı yapılır
    digitalWrite(REDLed,HIGH);
    digitalWrite(IRLed,LOW);
    n = 0;
    start = millis();
    reader = 0.;
    do
    {
      reader += analogRead (sensorPin);
      n++;
    }
    while (millis() < start + T);  
    reader /= n;  // ortalamamız var
    // En yeni olçum diziye atanır, en eski olçum duzuden çıkarılır.  
    // Boylece son olcumlerin ortalaması hesaplamaya doğru şekilde katılır
    ortRED -= okuRED[ptr];
    ortRED += reader;
    okuRED[ptr] = reader;
    sonRED = ortRED / orn_siz;
                    
    // R HESAPLANMASI
    // Hem IR hem de Kırmızı ledin bir periyorundaki tum ornekler kaydedilir.
    okuIRMM[ptrMM]=sonIR;
    okuREDMM[ptrMM]=sonRED;
    ptrMM++;
    ptrMM %= max_period;
    ornekSayaci++;
    
    // periyottaki tum ornekler kaydedildikten sonra max ve min değerler bulunur ve r hesaplanır
    
    if(ornekSayaci>=ornekler){
      ornekSayaci =0;
      IRmax = 0; IRmin=1023; REDmax = 0; REDmin=1023;
      for(int i=0;i<max_period;i++) {
        if( okuIRMM[i]> IRmax) IRmax = okuIRMM[i];
        if( okuIRMM[i]>0 && okuIRMM[i]< IRmin ) IRmin = okuIRMM[i];
        okuIRMM[i] =0;
        if( okuREDMM[i]> REDmax) REDmax = okuREDMM[i];
        if( okuREDMM[i]>0 && okuREDMM[i]< REDmin ) REDmin = okuREDMM[i];
        okuREDMM[i] =0;
      }
      R =  ( (REDmax-REDmin) / REDmin) / ( (IRmax-IRmin) / IRmin ) ;
    }
    float ortR;
    int ortBPM;  //ortBPM nabiz atışı ortalamasıdır
    float vSicaklik;
    int SpO2;

    // parmağın sensore yerşeip yerleşmediği kontrol edilir. eğer parmak yoksa kırmızı eğri kızılotesi eğrinin altındadır
    if (sonIR < 50 ) {
      parmakKontrol=false;
      delay(1000);
      Serial.println("" + String(lastortBPM) + "," + String(lastSPO2) + "," + String(lastvSicaklik) + "," + int(parmakKontrol));

    } 
    else{
       
       // sonIR dizideki değerlerin ortalamasını tutar
       // yukselen bir eğri var mı kontrol edilir (nabiz atışı)
       if (sonIR > oncekiIR)
       {
         yuks_say++;  // yukselen ornekler sayılır
         if (!yukselen && yuks_say > rise_threshold)
         {
            // yukselen bir eğri saptadık, bu bir nabiz atışına işaret eder.
            // son atıştan sonraki zaman kaydedilir, ve son 10 atım ortalama değer yakalamak için takip edilir
            // Yukselişi takip etmek aynı yukselimleri birden fazla kere olçmemizi engeller
            yukselen = true;
            olcumR[m] = R;
            olcumPeriod[m] = millis() - son_atim;
            son_atim = millis();
            int period = 0;
            for(int i =0; i<olcumler; i++){
            period += olcumPeriod[i];
            }
            // max ve min değerleri bulmak içinortalama periyot ve orneklerin sayısı hesaplanır
            period = period / olcumler;
            ornekler = period / (2*T);
            int ortPeriod = 0;
            int c = 0;
            // c son 10 atımdaki iyi olcumlerin adetini tutar 
            for(int i =1; i<olcumler; i++) {
              if ( (olcumPeriod[i] <  olcumPeriod[i-1] * 1.1)  &&  
                    (olcumPeriod[i] >  olcumPeriod[i-1] / 1.1)  ) {
                  c++;
                  ortPeriod += olcumPeriod[i];
                  ortR += olcumR[i];
               }
            }
            m++;
            m %= olcumler;      
            // gosterilen bpm ve R değerleri son 5 iyi atımın ortalamsı olarak hesaplanır
            ortBPM = 60000 / ( ortPeriod / c) ;
            ortR = ortR / c ;
            if(c > 4) {
              // SATuRASYON R'NİN BİR FONKSİYONUDUR. YANİ, KALİBRE EDİLMİŞ HALİDİR
              // Y = k*x + m
              // k ve m ise güvenilir bir başka oksimetre tarafından kalibre edilerek hesaplanır.
              // bizim kalibrasyonumuz:
              SpO2 = -23 * R + 103;
              if(ortBPM > 40 && ortBPM <220){
                if(SpO2 > 70 && SpO2 <150) {
                  vSicaklik = sicaklikSensor.getTemperature();
                  lastSPO2 = SpO2;
                  lastortBPM = ortBPM;
                  lastvSicaklik = vSicaklik;
                  parmakKontrol=true;
                  Serial.println("" + String(ortBPM) + "," + String(SpO2) + "," + String(vSicaklik) + "," + int(parmakKontrol));
                }
              }
            }else {
              if(c <3) continue; 
            }
           }
          }
          else
       {
         // Eğri düşmeye başlıyor
         yukselen = false;
         yuks_say = 0;
       }
       //Yeni değerle karşılaştırıp atım bulmak için
       oncekiIR = sonIR;
   }
   ptr++;
   ptr %= orn_siz;
   
   //Her 100 ölçümde bor ums ile uyarı vermek için: aşağıdaki kod kullanılabilir.

}}

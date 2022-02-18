/*
* This i a transmiter sketch for use with the ATmega32u4 and the RFM69CW radio module
* Author: Petrović Dejan
* Svet Kompjutera
* 18.2.2022
* 
*/
#include <SPI.h>
#include <RH_RF69.h>
/************ Radio Setup ***************/
#define RF69_FREQ 433.0
  #define RFM69_CS      8  //PB4
  #define RFM69_INT     7  //PE6
  #define RFM69_RST     4  //PD4
  
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
// Struct
struct dataStruct{
  float TcTemp ;
  float hum;
  float temp;
  float bat;
  unsigned long counter; 
}myData;
byte tx_buf[sizeof(myData)] = {0};
/*********** termistor ******************/
int tempPin = A1;  //PF6
int Vout;
float R1 = 10000; //otpornost na sobnoj temperaturi 10K ili 10000 oma
float R2, Tk, Tc; //otpornost, temperatura u K, temperatura u C
float Ac = 1.009249522e-03, Bc = 2.378405444e-04, Cc = 2.019202697e-07; //konstante
/*********** bat voltage *****************/
#define VBATPIN A0  //PF7
void setup() {
  //Serial.begin(9600);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    while (1);
  }
  rf69.setFrequency(RF69_FREQ));
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}
void loop() {
  readData();
  delay(1000);
  memcpy(tx_buf, &myData, sizeof(myData) );
  byte zize=sizeof(myData);
  rf69.send((uint8_t *)tx_buf, zize);
  myData.counter++;
  if(myData.counter > 100000){
    myData.counter=0;
    }
  rf69.waitPacketSent();
}
void readData(){
  Vout = analogRead(tempPin);
  R2 = R1 * (1023.0 / (float)Vout - 1.0); // konvertovanje iz analogne u digitalnu
  Tk = (1.0 / (Ac + Bc*log(R2) + Cc*log(R2)*log(R2)*log(R2))); // temperatura u kelvinima (K)
  Tc = Tk - 273.15; //temperatura u stepenima celzijusa
  myData.TcTemp=Tc;
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // Podelili smo sa 2 otpornika, pomnozimo nazad
  measuredvbat *= 3.3;  // Pomnozimo sa 3.3V, analogna referenca
  measuredvbat /= 1024; // Pretvaramo u napon
  myData.bat=measuredvbat;
}

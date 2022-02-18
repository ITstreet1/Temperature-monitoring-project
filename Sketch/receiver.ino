/*
* This i a receiver sketch for use with the ATmega32u4 and the RFM69CW radio module
* Author: Petrović Dejan
* Svet Kompjutera
* 18.2.2022
* 
*/
#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/************ Radio Setup ***************/
  #define RF69_FREQ 433.0
  #define RFM69_CS      8  //PB4
  #define RFM69_INT     7  //PE6
  #define RFM69_RST     4  //PD4
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
/********* Struct   ****************/
struct dataStruct{
  float TcTemp ;
  float hum;
  float temp;
  float bat;
  unsigned long counter;
}myData;
/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();
/*********** bat voltage *****************/
#define VBATPIN A0 //PF7
float measuredvbat;
/*********** LED ************************/
#define ledR 6  //PD7
#define ledG 13 //PC7
#define ledB 5  //PC6
#define rxTx 12 //PD6
int r,g,b = 0;

void setup() {
  //Serial.begin(9600);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();
  delay(500);
  oled.clearDisplay();
  oled.display();
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(rxTx, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  digitalWrite(ledB, LOW);
  digitalWrite(rxTx, LOW);
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
  rf69.setFrequency(RF69_FREQ);
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
    // OLED text display testovi
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.println("Booting... ");
  oled.display();
  delay(500);
}
void loop() {
 if (rf69.available()) {
    // Stigao paket 
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);
    if (rf69.recv(buf, &buflen))
    {
  int i;
  // Paket je ispravan, prikazi ga
  rf69.printBuffer("Got:", buf, buflen);
        memcpy(&myData, buf, sizeof(myData));
       /* Serial.println("");       
        Serial.print("TcTemp: ");
        Serial.print(myData.TcTemp);       
        Serial.print("  hum: ");
        Serial.print(myData.hum);
        Serial.print("  temp: ");
        Serial.print(myData.temp);
        Serial.print("  bat:  ");
        Serial.print(myData.bat);  
        Serial.print("  counter: ");
        Serial.println(myData.counter);*/
    }
  }
  measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // Podelili smo sa 2 otpornika, pomnozimo nazad
  measuredvbat *= 3.3;  // Pomnozimo sa 3.3V, analogna referenca
  measuredvbat /= 1024; // Pretvaramo u napon
  
  oled.clearDisplay();
  oled.setTextSize(3);
  oled.setTextColor(WHITE);
  oled.setCursor(0,5);
  oled.print((String)myData.TcTemp+" C");
  
  rgb();
  voltageBar();
  //ovo ne dirati
  oled.display();
}
void rgb(){
if(myData.TcTemp <= 20){
    r=0;
    g=0;
    b=1;
  }else if(myData.TcTemp > 20 && myData.TcTemp < 60){
    r=0;
    g=1;
    b=0;
  }else if(myData.TcTemp >= 60){
    r=1;
    g=0;
    b=0;
  }
  digitalWrite(ledB, b);
  digitalWrite(ledG, g);
  digitalWrite(ledR, r);
}
void voltageBar(){
  //tp4056 
  //overcharge detection 4.3V
  //overcharge release 4.1V
  //over-discharge detection 2.4V
  //over-discharge release 3.0V
  //charge current 1A
  int outVu = map(measuredvbat, 2.8, 4.3, 0, 128);
  oled.fillRect(0, 0, outVu, 2, SSD1306_WHITE);
  int outVd = map(myData.bat, 2.8, 4.3, 0, 128);
  oled.fillRect(0, 30, outVd, 2, SSD1306_WHITE);
}

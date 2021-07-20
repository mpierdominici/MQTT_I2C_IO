#include "Wire.h"
#include "PCF8574.h"
#include "DHT.h"
#include<Arduino.h>

#define SDA_PIN D1
#define SCL_PIN D2
#define PCF8574_ADRESS_WRITE 0x20
#define PCF8574_ADRESS_READ 0x21
#define BOARD_OUTPUT_CH_1 0
#define BOARD_OUTPUT_CH_2 1
#define BOARD_OUTPUT_CH_3 2
#define BOARD_INPUT_CH_1 3
#define BOARD_INPUT_CH_2 4
#define BOARD_INPUT_CH_3 5
#define BOARD_INPUT_CH_4 6
#define BOARD_INPUT_CH_5 7

#define BOARD_DHT11_PIN D7
#define DHTTYPE DHT11 

//PCF8574 pcf20(PCF8574_ADRESS_WRITE);
DHT dht(BOARD_DHT11_PIN, DHTTYPE);
PCF8574 expander;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
  //  pcf20.begin(SDA_PIN,SCL_PIN);
    dht.begin();
    expander.begin(0x20);
   
  /* Setup some PCF8574 pins for demo */
  expander.pinMode(0, OUTPUT);
  expander.pinMode(1, OUTPUT);
  expander.pinMode(2, OUTPUT);
  expander.pinMode(3, INPUT_PULLUP);

}
int i=0;
void loop() {
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
 // Serial.print(pcf20.readButton(BOARD_INPUT_CH_1));
 // Serial.print(pcf20.readButton(BOARD_INPUT_CH_2));
 // Serial.println(pcf20.readButton(BOARD_INPUT_CH_3));
  Serial.print("Temp :");
  Serial.print(t);
  Serial.print("  Hum :");
  Serial.println(h);
  //pcf20.write(i, true); 
  expander.digitalWrite(i,HIGH);


  delay(1000);
  //pcf20.write(i, false); 
  expander.digitalWrite(i,LOW);
  i++;
  i=i%3;
  

}

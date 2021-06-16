#include "Wire.h"
#include <PCF8574.h>
#include "DHT.h"

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

PCF8574 pcf20(PCF8574_ADRESS_WRITE);
DHT dht(BOARD_DHT11_PIN, DHTTYPE);


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pcf20.begin(SDA_PIN,SCL_PIN);
    dht.begin();

}
int i=0;
void loop() {
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  Serial.print(pcf20.readButton(BOARD_INPUT_CH_1));
  Serial.print(pcf20.readButton(BOARD_INPUT_CH_2));
  Serial.println(pcf20.readButton(BOARD_INPUT_CH_3));
  Serial.print("Temp :");
  Serial.print(t);
  Serial.print("  Hum :");
  Serial.println(h);
  pcf20.write(i, true); 



  delay(500);
  pcf20.write(i, false); 
  i++;
  i=i%3;

}

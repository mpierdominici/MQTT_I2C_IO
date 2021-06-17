#include "Wire.h"
#include <PCF8574.h>
#include "DHT.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define DEBUGG

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
#define BOARD_INPUT_NUMBER 5
#define SAMPLE_TIME_BUTTONS_MS 250
#define READ_INPUT_MASK 0x08 // mascara b0001000
#define BOARD_DHT11_PIN D7
#define DHTTYPE DHT11 

class memEdgeDetector
{
  public:
  memEdgeDetector();
  bool risingEdge(void);
  bool fallingEdge(void);
  void updateMme(bool newData);
  bool getState(void);

  bool newData;
  bool currentData;
  bool isRising;
  bool isFalling;
};

class myTimer
{
  public:
  myTimer(unsigned int seconds=0);
  bool timeOver(void);
  void setNewTime(unsigned long seconds_);
  void showInfo();
  
  unsigned long seconds;
  unsigned long startTime;
  void resetTimer(void);
    
};



char * ssid ="WIFI Pier2";
char * pass ="pagle736pagle";
unsigned int mqttPort=1883;

const char MqttUser[]="escritorioIO";
const char MqttPassword[]="1234";
const char MqttClientID[]="escIO";

IPAddress mqttServer(192,168,0,116);

WiFiClient wclient;
PubSubClient mqtt_client(wclient);
PCF8574 pcf20(PCF8574_ADRESS_WRITE); // instancia del modulo io i2c
DHT dht(BOARD_DHT11_PIN, DHTTYPE); //instancia del sensor de temperatura y humedad




void callback(char* topic, byte* payload, unsigned int length);
void  debug_message (char * string, bool newLine)
{
#ifdef DEBUGG
  if(string !=NULL)
  {
    if (!newLine)
    {
      Serial.print(string);
    }else
    {
      Serial.println(string);
    }
  }
  #endif
}

void setUpWifi(char * ssid, char * pass)
{
  String ip;
  debug_message(" ",true);
  debug_message(" ",true);
  debug_message("Conectandose a: ",false);
  debug_message(ssid,true);

  WiFi.begin(ssid,pass);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    debug_message(".",false);
  }
  debug_message(" ",true);
  debug_message("Coneccion realizada",true);
  debug_message("La ip es: ",false);
  ip=WiFi.localIP().toString();
  debug_message((char *)ip.c_str(),true);
}

void setUpMqtt(void)
{
  mqtt_client.setServer(mqttServer,mqttPort);
  mqtt_client.setCallback(callback);
}


void callback(char* topic, byte* payload, unsigned int length)
{
  int tiempo=0;
  payload[length]='\n';
  String message((char *)payload);
  debug_message("Llego un mensage, topic:",false);
  debug_message(topic,false);
  debug_message(", payload : ",false);
  debug_message((char *)payload,true);

  if(!strcmp(topic,"escritorio/salidas/ch1"))
  {
    //debug_message("LLEGO MENSAJE PARA DETENER LA BOMBA",true);
    
    pcf20.write(BOARD_OUTPUT_CH_1,((payload[0]-48)==1));
  }else if(!strcmp(topic,"escritorio/salidas/ch2")){
    pcf20.write(BOARD_OUTPUT_CH_2,((payload[0]-48)==1));
    
  }else if(!strcmp(topic,"escritorio/salidas/ch3")){
    pcf20.write(BOARD_OUTPUT_CH_3,((payload[0]-48)==1));
    
  }

}





void reconnect()
{
  while(!mqtt_client.connected())
  {
    debug_message("Intentando conectar al servidor MQTT",true);
    if (mqtt_client.connect(MqttClientID,MqttUser,MqttPassword))
      {
            debug_message("conectado",true);
  
  
            // ...suscrivirse a topicos
            mqtt_client.subscribe("escritorio/salidas/ch1");
            mqtt_client.subscribe("escritorio/salidas/ch2");
            mqtt_client.subscribe("escritorio/salidas/ch3");
            mqtt_client.subscribe("escritorio/sensor/getTempHum");
            mqtt_client.subscribe("escritorio/sensor/getInputs");
            
            


      }
      else
      {
        debug_message("intentando conetarse al broker",true);
        delay(3000);
      }
  }
}

void resetOutputsI2C (void){
    pcf20.write(BOARD_OUTPUT_CH_1,LOW);
    pcf20.write(BOARD_OUTPUT_CH_2,LOW);
    pcf20.write(BOARD_OUTPUT_CH_3,LOW);
}


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    pcf20.begin(SDA_PIN,SCL_PIN);
    dht.begin();
    setUpWifi(ssid,pass);
    setUpMqtt();
    resetOutputsI2C();
    

}
memEdgeDetector chInput[BOARD_INPUT_NUMBER];
myTimer sampleButtonsTimer(SAMPLE_TIME_BUTTONS_MS);
byte lecturaPcf8574 =0;
void loop() {
  if (!mqtt_client.connected()) 
      {
          resetOutputsI2C();
          reconnect();
      
      }
  mqtt_client.loop();
  if(sampleButtonsTimer.timeOver()){
    lecturaPcf8574=pcf20.read8();//leer las entradas
    for(int j=0;j<BOARD_INPUT_NUMBER;j++){
      String topic;
      topic=String("escritorio/sensor/input/ch");
      topic=topic+String(j+1);
      chInput[j].updateMme(lecturaPcf8574&(READ_INPUT_MASK<<j));
      if(chInput[j].fallingEdge()){
        mqtt_client.publish(topic.c_str(),"F");
      }
      if(chInput[j].risingEdge()){
        mqtt_client.publish(topic.c_str(),"R");
        
      }
      
    }
    
    
    //updetear los edge detector
    //mandar data    
  }
 
  //float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  //float t = dht.readTemperature();
  //chInput.updateMme(pcf20.readButton(BOARD_INPUT_CH_1));
  //Serial.print("Falling ");
  //Serial.print(chInput.fallingEdge());
  //Serial.print(" Rsing: ");
  //Serial.print(chInput.risingEdge());
  //Serial.print(" State: ");
  //Serial.println(chInput.getState());
  //Serial.print(pcf20.readButton(BOARD_INPUT_CH_2));
  //Serial.println(pcf20.readButton(BOARD_INPUT_CH_3));
  //Serial.print("Temp :");
  //Serial.print(t);
  //Serial.print("  Hum :");
  //Serial.println(h);
  //pcf20.write(i, true); 



  //delay(500);

}


//*************************EdgeDetector***********************

memEdgeDetector::memEdgeDetector()
{
  newData=0;
  currentData=0;
  isRising=0;
  isFalling=0;
}

bool memEdgeDetector::risingEdge(void){
  bool tempRes;
  tempRes=isRising;
  isRising=false;
  return tempRes;
}

bool memEdgeDetector::fallingEdge(void){
  bool tempRes;
  tempRes=isFalling;
  isFalling=false;
  return tempRes;
}

bool memEdgeDetector::getState(void){
  return currentData;
}

void memEdgeDetector::updateMme(bool newData){
  this->newData=newData;

  if(currentData==false && this->newData==true){
    isRising=true;
  }else if(currentData==true && this->newData==false){
    isFalling=true;    
  }
  currentData=this->newData;

  
}


//***********************TIMER**********************************



myTimer::myTimer(unsigned int seconds)
{
  setNewTime(seconds);
}

//timeOver
//devuelve true si ya paso el tiempo seteado,
//caso contrario devuelve false
//
bool myTimer::timeOver(void)
{
  if((millis())>startTime)
  {
    resetTimer();
    return true;
  }
  else
  {
    return false;
  }
}

void myTimer::resetTimer(void)
{
  unsigned long temp=seconds+millis();
 
  startTime=temp;
  //Serial.print("se llamo a rest timer con: ");
  //Serial.println(startTime);
}

void  myTimer::setNewTime(unsigned long seconds_)
{
  //unsigned long temp=1000*seconds_; //tiempo en sefundos
  unsigned long temp=seconds_; //tiempo en milisegundos
  //Serial.println(temp);
  seconds=temp;
 
  //Serial.print("s seteo un timer cada: ");
  //Serial.print(seconds_);
  //Serial.print(" se registro un tirmpo de: ");
  //Serial.println(seconds/1000);
  resetTimer();

}

void myTimer::showInfo()
{
  //Serial.println(startTime);
  unsigned long dif=startTime-millis();
  //Serial.print("Remaining time (seconds):");
  //Serial.println(dif/1000);
  //Serial.println(startTime);
  //Serial.println(millis());
  //Serial.println(seconds/1000);
}

#include "Arduino.h"
#include <Timer.h>
#include <PCF8563TimeManager.h>
#include <SPI.h>
#include <Esp32SecretManager.h>
#include <FastLED.h>
#include <PanchoVisualizerWifiManager.h>
#include <PanchoTankFlowData.h>
#include <TM1637Display.h>
#include <Arduino_JSON.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <HTTPClient.h>

#include <Arduino_JSON.h>

// #define LORA_M0 14
// #define LORA_M1 13
// #define LORA_AUX 15
bool loraActive = false;
uint8_t currentFunction=0;
uint8_t currentAnimateLightCounter=4;
#define REFRESH_TIMER_SECONDS 4
Timer refreshTimer(REFRESH_TIMER_SECONDS);
HTTPClient http;

#define TEMPERATURE 27
#define UI_CLK 33
#define UI2_DAT 25
#define UI3_DAT 26
#define UI4_DAT 5

//
// for build 9 and above
 //#define UI5_DAT 18
//
// for build 8 and below
//
#define UI5_DAT 4

#define UI6_DAT 23
#define UI7_DAT 32



#define RTC_BATT_VOLT 36
#define LED_PIN 19
#define NUM_LEDS 8
#define OP_MODE 34

//LoRa_E22 e22ttl(&Serial2);
//LoRa_E22 e22ttl(&Serial2, LORA_AUX, LORA_M0, LORA_M1); // AUX M0 M1

PanchoConfigData panchoConfigData;
PanchoTankFlowData panchoTankFlowData;
String serialNumber;
uint8_t delayTime = 10;
TM1637Display display1(UI_CLK, UI2_DAT);
TM1637Display display2(UI_CLK, UI5_DAT);
TM1637Display display3(UI_CLK, UI3_DAT);
TM1637Display display4(UI_CLK, UI6_DAT);
TM1637Display display5(UI_CLK, UI4_DAT);
TM1637Display display6(UI_CLK, UI7_DAT);

CRGB leds[NUM_LEDS];
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool isHost = true;
#define UNIQUE_ID_SIZE 8
//
// for Pancho build 8 and below is 18
// from build 9 onwards is 4
//
#define RTC_CLK_OUT 18 //4

OneWire oneWire(TEMPERATURE);
DallasTemperature tempSensor(&oneWire);

String stationName;
uint8_t secondsSinceLastDataSampling = 0;
PCF8563TimeManager timeManager(Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);

PanchoVisualizerWifiManager wifiManager(Serial, timeManager, secretManager, panchoTankFlowData, panchoConfigData);
bool wifiActive = false;
bool apActive = false;

int badPacketCount = 0;
byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0xFF;  // address of this device
byte destination = 0xAA;   // destination to send to


long lastPulseTime = 0;
uint8_t uniqueId[UNIQUE_ID_SIZE];
long lastMillis;
uint8_t SECONDOFFSET = 10;
uint8_t timeZoneHours = 10;
static byte monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
String currentIpAddress = "No IP";
bool inPulse = true;


long lastTimeUpdateMillis = 0;
RTCInfoRecord currentTimerRecord;
#define TIME_RECORD_REFRESH_SECONDS 3
volatile bool clockTicked = false;

struct DisplayData {
  int value;
  int dp;
} displayData;
//
// the dispalay urls
//
/*
//String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%201%20TimeZoneTimestamp"
String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%201%20Cloudiness";
//String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%202%20TimeZoneTimestamp"
String display2URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%202%20Cloudiness";
//String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%203%20TimeZoneTimestamp"
String display3URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%203%20Cloudiness";
//String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%204%20TimeZoneTimestamp"
String display4URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%204%20Cloudiness";
//String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%205%20TimeZoneTimestamp"
String display5URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%205%20Cloudiness";
//String display1URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%206%20TimeZoneTimestamp"
String display6URL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%206%20Cloudiness";
*/
//const char* display1URL = "http://192.168.1.174/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Battery%20Voltage";
// const char* display2URL = "http://192.168.1.174/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Charge";
// const char* display3URL = "http://192.168.1.174/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Load";
// const char* display4URL = "http://192.168.1.174/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Operational%20Data:Generator:Generator%20Running%20Time";

const char* display1URL = "http://Ra.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Battery%20Voltage";
const char* display2URL = "http://Ra.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Charge";
const char* display3URL = "http://Ra.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Load";
const char* display4URL = "http://Ra.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Operational%20Data:Generator:Generator%20Running%20Time";
const char* display5URL = "http://Sento.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Sento:Purpose:Sensor%20Data:Tub%20Temperature:TubTemperature";
const char* display6URL = "http://Sento.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Sento:Purpose:Sensor%20Data:Heat%20Exchange%20Temperature:HeatExchangeTemperature";


String display1CloudURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%201%20Cloudiness";
String display2CloudURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%202%20Cloudiness";
String display3CloudURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%203%20Cloudiness";
String display4CloudURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%204%20Cloudiness";
String display5CloudURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%205%20Cloudiness";


String display1TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%201%20Temperature";
String display2TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%202%20Temperature";
String display3TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%203%20Temperature";
String display4TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%204%20Temperature";
String display5TempURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%205%20Temperature";

String display1RainURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%201%20Rain";
String display2RainURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%202%20Rain";
String display3RainURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%203%20Rain";
String display4RainURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%204%20Rain";
String display5RainURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%205%20Rain";


String display1HumURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%201%20Humidity";
String display2HumURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%202%20Humidity";
String display3HumURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%203%20Humidity";
String display4HumURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%204%20Humidity";
String display5HumURL = "http://Tlaloc.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Tlaloc:Purpose:Sensor%20Data:Short%20Term%20Weather%20Forecast:Short%20Term%20Forecast%20Period%205%20Humidity";

String display1SentoTubURL = "http://Sento.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Sento:Purpose:Sensor%20Data:Tub%20Temperature:TubTemperature";
String display2SentoHeatURL = "http://Sento.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Sento:Purpose:Sensor%20Data:Heat%20Exchange%20Temperature:HeatExchangeTemperature";


JSONVar jsonData;
//
// interrupt functions
//


void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}

//
// end of interrupt functions
//


String getSensorData() {
  String toReturn = "";

  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.secondsTime);

  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.temperature);

  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.rssi);
  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.snr);
  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.operatingStatus);
  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.sleepPingMinutes);
  toReturn += (F("#"));
  toReturn += (panchoTankFlowData.secondsSinceLastPulse);
  toReturn += (F("#"));
  toReturn += wifiManager.getRSSI();
  toReturn += (F("#"));
  toReturn += wifiManager.getIpAddress();
  toReturn += (F("#"));
  toReturn += wifiManager.getSoft_ap_ssid();
  toReturn += (F("#"));
  toReturn += wifiManager.getApAddress();
  toReturn += (F("#"));
  toReturn += wifiManager.getHostName();
  toReturn += (F("#"));
  toReturn += serialNumber;


  return toReturn;
}

void updateServerStatus(){
    // server status url returns:
    //{"Status":"Success","Message":"","Time":"2023-02-23 20:03:55.112","Data":"{\"servers\":[{\"instanceid\":2781656802463868147,\"name\":\"digital-stables\",\"externalIpAddress\":\"34.129.116.33\",\"internalIpAddress\":\"10.192.0.3\",\"status\":\"TERMINATED\"},{\"instanceid\":7870752577853258605,\"name\":\"digital-stables-validation\",\"internalIpAddress\":\"10.192.0.2\",\"status\":\"TERMINATED\"}]}"}
    const uint8_t ds[] = {
        SEG_B | SEG_B | SEG_D | SEG_E | SEG_G,          // D
        SEG_A |SEG_C |SEG_D | SEG_F | SEG_G             // S
      };
      display1.clear();
      display2.clear();
      display3.clear();
      display4.clear();
      display5.clear();
      display6.clear();
      display1.setSegments(ds, 2, 0);
     

    String serverStatusURL="http://192.168.1.137/FactoryServlet?formName=GetSimpleServerStatus";
    http.begin(serverStatusURL.c_str());
    int httpCode = http.GET();
    Serial.print("server monitoring httpCode= ");
      Serial.println(httpCode);
    if (httpCode  == 200) {
      String payload = http.getString();
      Serial.print("server monitoring returns ");
      Serial.println(payload);
      JSONVar statusData = JSON.parse(payload);
      

      const uint8_t on[] = {
        SEG_E | SEG_G | SEG_C | SEG_D,  // o
        SEG_C | SEG_E | SEG_G          // n    
      };
    
      const uint8_t off[] = {
        SEG_E | SEG_G | SEG_C | SEG_D,  // o
        SEG_A | SEG_G | SEG_E | SEG_F,  // F
        SEG_A | SEG_G | SEG_E | SEG_F  // F
      };
      uint8_t dsstatus = (int)statusData["digital-stables"];
      const uint8_t dss[] = {
        SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,          // D
      SEG_A |SEG_C |SEG_D | SEG_F | SEG_G,             // S
        SEG_A |SEG_C |SEG_D | SEG_F | SEG_G             // S
      };
      display3.clear();
      display3.setSegments(dss, 3, 0);

      display4.clear();
      if(dsstatus==0){
        display4.setSegments(off, 3, 0);
        leds[2] = CRGB(0, 0, 0);
      }else if(dsstatus==1){
        display4.setSegments(on, 2, 0);
        leds[2] = CRGB(0, 255, 0);
      }
        uint8_t dsvalstatus = (int)statusData["digital-stables-validation"];
        uint8_t dsval = (int)statusData["digital-stables-validation"];
      const uint8_t dsv[] = {
        SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          // U
        SEG_A | SEG_B | SEG_C | SEG_E | SEG_F| SEG_G,  // A
        SEG_F | SEG_E | SEG_D                           //L
      };
      display5.clear();
      display5.setSegments(dsv, 3, 0);

      display6.clear();
      if(dsvalstatus==0){
        display6.setSegments(off, 3, 0);
        leds[3] = CRGB(0, 0, 0);
      }else if(dsvalstatus==1){
        display6.setSegments(on, 2, 0);
        leds[3] = CRGB(0, 255, 0);
      }
      FastLED.show();
      http.end();
    }else{
      leds[2] = CRGB(255, 0, 0);
      leds[3] = CRGB(255, 0, 0);
      display2.showNumberDec(httpCode,false);
    }
}

void updateDisplaysCloud() {



  int value1 = processDisplayValue(display1CloudURL, &displayData);
  int dp1 = displayData.dp;
  int value2 = processDisplayValue(display2CloudURL, &displayData);
  int dp2 = displayData.dp;
  int value3 = processDisplayValue(display3CloudURL, &displayData);
  int dp3 = displayData.dp;
  int value4 = processDisplayValue(display4CloudURL, &displayData);
  int dp4 = displayData.dp;
  int value5 = processDisplayValue(display5CloudURL, &displayData);
  int dp5 = displayData.dp;

  if (dp1 > 0) {
    display2.showNumberDecEx(value1, (0x80 >> dp1), false);
  } else {
    display2.showNumberDec(value1, false);
  }
  delay(20);



  if (dp2 > 0) {
    display3.showNumberDecEx(value2, (0x80 >> dp2), false);
  } else {
    display3.showNumberDec(value2, false);
  }
  delay(20);


  if (dp3 > 0) {
    display4.showNumberDecEx(value3, (0x80 >> dp3), false);
  } else {
    display4.showNumberDec(value3, false);
  }
  delay(20);


  if (dp4 > 0) {
    display5.showNumberDecEx(value4, (0x80 >> dp4), false);
  } else {
    display5.showNumberDec(value4, false);
  }
  delay(20);


  if (dp5 > 0) {
    display6.showNumberDecEx(value5, (0x80 >> dp5), false);
  } else {
    display6.showNumberDec(value5, false);
  }
  delay(20);
  const uint8_t cloud[] = {
    SEG_A | SEG_D | SEG_E | SEG_F,                  // C
    SEG_D | SEG_E | SEG_F,                          // L
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  // O
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_F           // U
  };
  display1.clear();
  display1.setSegments(cloud, 4, 0);
}

void updateDisplaysRain() {

  int value1 = processDisplayValue(display1RainURL, &displayData);
  int dp1 = displayData.dp;
  int value2 = processDisplayValue(display2RainURL, &displayData);
  int dp2 = displayData.dp;
  int value3 = processDisplayValue(display3RainURL, &displayData);
  int dp3 = displayData.dp;
  int value4 = processDisplayValue(display4RainURL, &displayData);
  int dp4 = displayData.dp;
  int value5 = processDisplayValue(display5RainURL, &displayData);
  int dp5 = displayData.dp;

  if (dp1 > 0) {
    display2.showNumberDecEx(value1, (0x80 >> dp1), false);
  } else {
    display2.showNumberDec(value1, false);
  }
  delay(20);



  if (dp2 > 0) {
    display3.showNumberDecEx(value2, (0x80 >> dp2), false);
  } else {
    display3.showNumberDec(value2, false);
  }
  delay(20);


  if (dp3 > 0) {
    display4.showNumberDecEx(value3, (0x80 >> dp3), false);
  } else {
    display4.showNumberDec(value3, false);
  }
  delay(20);


  if (dp4 > 0) {
    display5.showNumberDecEx(value4, (0x80 >> dp4), false);
  } else {
    display5.showNumberDec(value4, false);
  }
  delay(20);


  if (dp5 > 0) {
    display6.showNumberDecEx(value5, (0x80 >> dp5), false);
  } else {
    display6.showNumberDec(value5, false);
  }
  delay(20);
  const uint8_t rain[] = {
    SEG_E | SEG_G,                                  // r
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // A
    SEG_E | SEG_F,                                  // I
    SEG_C | SEG_E | SEG_G                           // n
  };
  display1.clear();
  display1.setSegments(rain, 4, 0);
}

void updateDisplaysTemp() {
  int value1 = processDisplayValue(display1TempURL, &displayData);
  int dp1 = displayData.dp;
  int value2 = processDisplayValue(display2TempURL, &displayData);
  int dp2 = displayData.dp;
  int value3 = processDisplayValue(display3TempURL, &displayData);
  int dp3 = displayData.dp;
  int value4 = processDisplayValue(display4TempURL, &displayData);
  int dp4 = displayData.dp;
  int value5 = processDisplayValue(display5TempURL, &displayData);
  int dp5 = displayData.dp;

  if (dp1 > 0) {
    display2.showNumberDecEx(value1, (0x80 >> dp1), false);
  } else {
    display2.showNumberDec(value1, false);
  }
  delay(20);



  if (dp2 > 0) {
    display3.showNumberDecEx(value2, (0x80 >> dp2), false);
  } else {
    display3.showNumberDec(value2, false);
  }
  delay(20);


  if (dp3 > 0) {
    display4.showNumberDecEx(value3, (0x80 >> dp3), false);
  } else {
    display4.showNumberDec(value3, false);
  }
  delay(20);


  if (dp4 > 0) {
    display5.showNumberDecEx(value4, (0x80 >> dp4), false);
  } else {
    display5.showNumberDec(value4, false);
  }
  delay(20);


  if (dp5 > 0) {
    display6.showNumberDecEx(value5, (0x80 >> dp5), false);
  } else {
    display6.showNumberDec(value5, false);
  }
  delay(20);
  const uint8_t temp[] = {
    SEG_D | SEG_E | SEG_F | SEG_G,         // t
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G  // E
  };
  display1.clear();
  display1.setSegments(temp, 2, 0);
}

void updateDisplaysHumidity() {
  int value1 = processDisplayValue(display1HumURL, &displayData);
  int dp1 = displayData.dp;
  int value2 = processDisplayValue(display2HumURL, &displayData);
  int dp2 = displayData.dp;
  int value3 = processDisplayValue(display3HumURL, &displayData);
  int dp3 = displayData.dp;
  int value4 = processDisplayValue(display4HumURL, &displayData);
  int dp4 = displayData.dp;
  int value5 = processDisplayValue(display5HumURL, &displayData);
  int dp5 = displayData.dp;

  if (dp1 > 0) {
    display2.showNumberDecEx(value1, (0x80 >> dp1), false);
  } else {
    display2.showNumberDec(value1, false);
  }
  delay(20);



  if (dp2 > 0) {
    display3.showNumberDecEx(value2, (0x80 >> dp2), false);
  } else {
    display3.showNumberDec(value2, false);
  }
  delay(20);


  if (dp3 > 0) {
    display4.showNumberDecEx(value3, (0x80 >> dp3), false);
  } else {
    display4.showNumberDec(value3, false);
  }
  delay(20);


  if (dp4 > 0) {
    display5.showNumberDecEx(value4, (0x80 >> dp4), false);
  } else {
    display5.showNumberDec(value4, false);
  }
  delay(20);


  if (dp5 > 0) {
    display6.showNumberDecEx(value5, (0x80 >> dp5), false);
  } else {
    display6.showNumberDec(value5, false);
  }
  delay(20);
  const uint8_t hum[] = {
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // H
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_F   // U
  };
  display1.clear();
  display1.setSegments(hum, 2, 0);
}

void updateDisplaysRa() {
  int value1 = processDisplayValue(display1URL, &displayData);
  int dp1 = displayData.dp;
  int value2 = processDisplayValue(display2URL, &displayData);
  int dp2 = displayData.dp;
  int value3 = processDisplayValue(display3URL, &displayData);
  int dp3 = displayData.dp;
  int value4 = processDisplayValue(display4URL, &displayData);
  int dp4 = displayData.dp;
  int value5 = processDisplayValue(display5URL, &displayData);
  int dp5 = displayData.dp;
  float rv = value1;
  if(value1>1000){
    rv=value1/100;
  }
  if(rv<47){
    leds[1] = CRGB(255, 0, 0);
  }else if(rv>=47 && rv<49){
    leds[1] = CRGB(255, 255, 0);
  }else if(rv>=49 && rv<52){
    leds[1] = CRGB(0, 255, 0);
  }else if(rv>=52 && rv<54.80){
    leds[1] = CRGB(0, 0, 255);
  }else if(rv>=54.80){
    leds[1] = CRGB(0, 255, 255);
  }
  FastLED.show();
  if (dp1 > 0) {
    display2.showNumberDecEx(value1, (0x80 >> dp1), false);
  } else {
    display2.showNumberDec(value1, false);
  }
  delay(20);



  if (dp2 > 0) {
    display3.showNumberDecEx(value2, (0x80 >> dp2), false);
  } else {
    display3.showNumberDec(value2, false);
  }
  delay(20);


  if (dp3 > 0) {
    display4.showNumberDecEx(value3, (0x80 >> dp3), false);
  } else {
    display4.showNumberDec(value3, false);
  }
  delay(20);


  if (dp4 > 0) {
    display5.showNumberDecEx(value4, (0x80 >> dp4), false);
  } else {
    display5.showNumberDec(value4, false);
  }
  delay(20);


  if (dp5 > 0) {
    display6.showNumberDecEx(value5, (0x80 >> dp5), false);
  } else {
    display6.showNumberDec(value5, false);
  }
  delay(20);

  display6.clear();
  delay(100);
  const uint8_t ra[] = {
    SEG_E | SEG_G,                                 // R
    SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F  // a
  };
  display1.clear();
  display1.setSegments(ra, 2, 0);
}



void updateDisplaysSento() {
  int value1 = processDisplayValue(display1SentoTubURL, &displayData);
  int dp1 = displayData.dp;
  int value2 = processDisplayValue(display2SentoHeatURL, &displayData);
  int dp2 = displayData.dp;
 
  if (dp1 > 0) {
    display2.showNumberDecEx(value1, (0x80 >> dp1), false);
  } else {
    display2.showNumberDec(value1, false);
  }
  delay(20);



  if (dp2 > 0) {
    display3.showNumberDecEx(value2, (0x80 >> dp2), false);
  } else {
    display3.showNumberDec(value2, false);
  }
  delay(20);

  display4.clear();
  display5.clear();
  display6.clear();
  delay(100);
  const uint8_t sento[] = {
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,  //S
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G , // E
    SEG_C | SEG_E | SEG_G ,                // n
    SEG_D | SEG_E | SEG_F | SEG_G,         // t
    
  };

  display1.clear();
  display1.setSegments(sento, 4, 0);
}

void updateDisplays() {
  for (int i = 4; i < 8; i++) {
    leds[i] = CRGB(255, 0, 255);
  }
  FastLED.show();
  int value1 = processDisplayValue(display1URL, &displayData);

  if (displayData.dp > 0) {
    display1.showNumberDecEx(value1, (0x80 >> displayData.dp), false);
  } else {
    display1.showNumberDec(value1, false);
  }
  delay(100);


  int value2 = processDisplayValue(display2URL, &displayData);

  if (displayData.dp > 0) {
    display2.showNumberDecEx(value2, (0x80 >> displayData.dp), false);
  } else {
    display2.showNumberDec(value2, false);
  }
  delay(100);

  int value3 = processDisplayValue(display3URL, &displayData);

  if (displayData.dp > 0) {
    display3.showNumberDecEx(value3, (0x80 >> displayData.dp), false);
  } else {
    display3.showNumberDec(value3, false);
  }
  delay(100);

  int value4 = processDisplayValue(display4URL, &displayData);

  if (displayData.dp > 0) {
    display4.showNumberDecEx(value4, (0x80 >> displayData.dp), false);
  } else {
    display4.showNumberDec(value4, false);
  }
  delay(100);

  int value5 = processDisplayValue(display5URL, &displayData);

  if (displayData.dp > 0) {
    display5.showNumberDecEx(value5, (0x80 >> displayData.dp), false);
  } else {
    display5.showNumberDec(value5, false);
  }
  delay(100);

  int value6 = processDisplayValue(display6URL, &displayData);
  if (displayData.dp > 0) {
    display6.showNumberDecEx(value6, (0x80 >> displayData.dp), false);
  } else {
    display6.showNumberDec(value6, false);
  }

  delay(100);

  for (int i = 4; i < 8; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

int processDisplayValue(String displayURL, struct DisplayData* displayData) {
  int value = 0;
  bool debug = false;
  // Serial.print("getting data for ");
  //  Serial.println(displayURL);

  String displayValue = wifiManager.getTeleonomeData(displayURL, debug);
  //   Serial.print("received ");
  //  Serial.print(displayValue);
  if (displayValue.indexOf("Error") > 0) {
    value = 9999;
  } else {
    jsonData = JSON.parse(displayValue);
    if (jsonData["Value Type"] == JSONVar("int")) {
      auto val = (const char*)jsonData["Value"];
      if (val == NULL) {
        value = (int)jsonData["Value"];
      } else {
        String s((const char*)jsonData["Value"]);
        value = s.toInt();
      }
      displayData->dp = -1;
      ///  Serial.print("int value= ");
      // Serial.println(value);
    } else if (jsonData["Value Type"] == JSONVar("double")) {
      auto val = (const char*)jsonData["Value"];
      if (val == NULL) {
        double valueF = (double)jsonData["Value"];
        if (valueF == (int)valueF) {
          value = (int)valueF;
          displayData->dp = -1;
        } else {
          value = (int)(100 * valueF);
          displayData->dp = 1;
        }
      } else {
        String s((const char*)jsonData["Value"]);
        float valueF = s.toFloat();
        if (valueF == (int)valueF) {
          value = (int)valueF;
          displayData->dp = -1;
        } else {
          value = (int)(100 * valueF);
          displayData->dp = 1;
        }
      }
    } else {
      value = 9997;
      displayData->dp = -1;
    }
  }
  displayData->value = value;

  return value;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(RTC_CLK_OUT, INPUT_PULLUP);
  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt%20Pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt%20To set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 0);
  }
  FastLED.show();
  timeManager.start();
  timeManager.PCF8563osc1Hz();

  tempSensor.begin();
  uint8_t address[8];
  tempSensor.getAddress(address, 0);
  for (uint8_t i = 0; i < 8; i++) {
    //if (address[i] < 16) Serial.print("0");
    serialNumber += String(address[i], HEX);
  }

  Serial.print("serial number:");
  Serial.println(serialNumber);

  display1.setBrightness(0x0f);
  display2.setBrightness(0x0f);
  display3.setBrightness(0x0f);
  display4.setBrightness(0x0f);
  display5.setBrightness(0x0f);
  display6.setBrightness(0x0f);
  display1.clear();
  display2.clear();
  display3.clear();
  display4.clear();
  display5.clear();
  display6.clear();





  wifiManager.start();
  bool stationmode = wifiManager.getStationMode();
  Serial.print("Starting wifi stationmode=");
  Serial.println(stationmode);

  //  serialNumber = wifiManager.getMacAddress();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  if (stationmode) {
    ipAddress = wifiManager.getIpAddress();
    Serial.print("ipaddress=");
    Serial.println(ipAddress);

    if (ipAddress == "" || ipAddress == "0.0.0.0") {
      for (int i = 0; i < 4; i++) {
        leds[i] = CRGB(255, 0, 0);
      }
      for (int i = 4; i < 8; i++) {
        leds[i] = CRGB(255, 255, 0);
      }
      FastLED.show();
      setApMode();


      // wifiManager.restartWifi();
      // ipAddress = wifiManager.getIpAddress();
      //  Serial.print("2-ipaddress=");
      // Serial.println(ipAddress);
      // if (ipAddress == ""|| ipAddress == "0.0.0.0") {
      //   setApMode();
      // } else {
      //   setStationMode(ipAddress);
      // }

    } else {
      setStationMode(ipAddress);
    }
  } else {
    setApMode();
  }


  pinMode(RTC_BATT_VOLT, INPUT);
  pinMode(OP_MODE, INPUT_PULLUP);


  currentTimerRecord = timeManager.now();
  int time = currentTimerRecord.hour * 100 + currentTimerRecord.minute;
  display1.showNumberDecEx(time, (0x80 >> 1), false);
  delay(1000);

  
  refreshTimer.start();

  Serial.println("Ok-Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  bool opmode = false;
  // put your main code here, to run repeatedly:
  if (clockTicked) {
    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    refreshTimer.tick();
   currentTimerRecord = timeManager.now();
    wifiManager.setCurrentTimerRecord(currentTimerRecord);
   currentAnimateLightCounter++;
    
    opmode = digitalRead(OP_MODE);
    bool weatherStationMode = true;
//Serial.println(currentFunction);
    if (wifiActive) {
      if ( weatherStationMode) {
        if(refreshTimer.status()){
          //if (currentTimerRecord.second == 0 || currentTimerRecord.second == 30) {
          if (currentFunction==0) {
            animateLights(255, 102, 0 );
            updateDisplaysRa();
          }else if (currentFunction==1) {
            animateLights(0, 255, 255 );
            updateDisplaysSento();
          } else if (currentFunction==2) {//(currentTimerRecord.second == 6 || currentTimerRecord.second == 36) {
            animateLights(255, 0, 255 );
            updateDisplaysCloud();
          } else if (currentFunction==3) {//(currentTimerRecord.second == 12 || currentTimerRecord.second == 42) {
            updateDisplaysTemp();
          } else if (currentFunction==4) {//(currentTimerRecord.second == 18 || currentTimerRecord.second == 48) {
            updateDisplaysRain();
          } else if (currentFunction==5) {//(currentTimerRecord.second == 24 || currentTimerRecord.second == 54) {
            updateDisplaysHumidity();
          }else if (currentFunction==6) {//(currentTimerRecord.second == 24 || currentTimerRecord.second == 54) {
           animateLights(57, 181, 74 );
            updateServerStatus();
          }
          currentFunction++;
          if(currentFunction>6){
            currentFunction=0;
            currentAnimateLightCounter=4;
          }
          refreshTimer.reset();
        }
      } else {
        if (currentTimerRecord.second == 0 || currentTimerRecord.second == 30) {
          updateDisplays();
          if (currentTimerRecord.minute == 0) {
            //		Serial.println(F("New Hour"));
            if (currentTimerRecord.hour == 0) {
              //	Serial.println(F("New Day"));
            }
          }
        } else if (currentTimerRecord.second == 8 || currentTimerRecord.second == 38) {

          leds[4] = CRGB(0, 255, 0);
          leds[5] = CRGB(0, 0, 0);
          leds[6] = CRGB(0, 0, 0);
          FastLED.show();
        } else if (currentTimerRecord.second == 16 || currentTimerRecord.second == 46) {

          leds[4] = CRGB(0, 255, 0);
          leds[5] = CRGB(0, 255, 0);
          leds[6] = CRGB(0, 0, 0);
          FastLED.show();
        } else if (currentTimerRecord.second == 24 || currentTimerRecord.second == 54) {

          leds[4] = CRGB(0, 255, 0);
          leds[5] = CRGB(0, 255, 0);
          leds[6] = CRGB(0, 255, 0);
          FastLED.show();
        }
      }
    }  
  }
  if (Serial.available() != 0) {
    String command = Serial.readString();
    Serial.print(F("command="));
    Serial.println(command);
    if (command.startsWith("Ping")) {
      Serial.println(F("Ok-Ping"));
    } else if (command.startsWith("ScanNetworks")) {
      wifiManager.scanNetworks();
    } else if (command.startsWith("GetWifiStatus")) {


      uint8_t status = wifiManager.getWifiStatus();
      Serial.print("WifiStatus=");
      Serial.println(status);


      Serial.println("Ok-GetWifiStatus");

    } else if (command.startsWith("ConfigWifiSTA")) {
      //ConfigWifiSTA#ssid#password
      //ConfigWifiSTA#MainRouter24##Office#
      //ConfigWifiSTA#LivingRoom##KitchenPanel#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid, password, hostname);
      if (staok) {
        leds[0] = CRGB(0, 0, 255);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");

    } else if (command.startsWith("ConfigWifiAP")) {
      //ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostaname
      //ConfigWifiAP#pancho5##pancho5

      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);

      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      if (stat) {
        leds[0] = CRGB(0, 255, 0);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiAP");

    } else if (command.startsWith("GetOperationMode")) {
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        Serial.println(F("PGM"));
      } else {
        Serial.println(F("RUN"));
      }
    } else if (command.startsWith("SetTime")) {
      //SetTime#24#10#19#4#17#32#00
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        timeManager.setTime(command);
        Serial.println("Ok-SetTime");
      } else {
        Serial.println("Failure-SetTime");
      }

    } else if (command.startsWith("SetFieldId")) {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();
    } else if (command.startsWith("GetTime")) {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
      Serial.println("Ok-GetTime");
      Serial.flush();
    } else if (command.startsWith("GetCommandCode")) {
      long code = 123456;  //secretManager.generateCode();
      //
      // patch a bug in the totp library
      // if the first digit is a zero, it
      // returns a 5 digit number
      if (code < 100000) {
        Serial.print("0");
        Serial.println(code);
      } else {
        Serial.println(code);
      }

      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("VerifyUserCode")) {
      String codeInString = generalFunctions.getValue(command, '#', 1);
      long userCode = codeInString.toInt();
      boolean validCode = true;  //secretManager.checkCode( userCode);
      String result = "Failure-Invalid Code";
      if (validCode) result = "Ok-Valid Code";
      Serial.println(result);
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetSecret")) {
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        //  char secretCode[SHARED_SECRET_LENGTH];
        String secretCode = secretManager.readSecret();
        Serial.println(secretCode);
        Serial.println("Ok-GetSecret");
      } else {
        Serial.println("Failure-GetSecret");
      }
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SetSecret")) {
      uint8_t switchState = digitalRead(OP_MODE);
      if (switchState == LOW) {
        //SetSecret#IZQWS3TDNB2GK2LO#6#30
        String secret = generalFunctions.getValue(command, '#', 1);
        int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
        int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
        secretManager.saveSecret(secret, numberDigits, periodSeconds);
        Serial.println("Ok-SetSecret");
        Serial.flush();
        delay(delayTime);
      } else {
        Serial.println("Failure-SetSecret");
      }


    } else if (command == "Flush") {
      while (Serial.read() >= 0)
        ;
      Serial.println("Ok-Flush");
      Serial.flush();
    } else if (command.startsWith("PulseStart")) {
      inPulse = true;
      Serial.println("Ok-PulseStart");
      Serial.flush();
      delay(delayTime);

    } else if (command.startsWith("PulseFinished")) {
      inPulse = false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
      delay(delayTime);

    } else if (command.startsWith("IPAddr")) {
      currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SSID")) {
      String currentSSID = generalFunctions.getValue(command, '#', 1);
      wifiManager.setCurrentSSID(currentSSID.c_str());
      Serial.println("Ok-currentSSID");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetIpAddress")) {
      Serial.println(wifiManager.getIpAddress());
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("RestartWifi")) {
      wifiManager.restartWifi();
      Serial.println("Ok-restartWifi");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("HostMode")) {
      Serial.println("Ok-HostMode");
      Serial.flush();
      delay(delayTime);
      isHost = true;
    } else if (command.startsWith("NetworkMode")) {
      Serial.println("Ok-NetworkMode");
      Serial.flush();
      delay(delayTime);
      isHost = false;
    } else if (command.startsWith("GetSensorData")) {


      Serial.print(getSensorData());
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("AsyncData")) {
      Serial.print("AsyncCycleUpdate#");
      Serial.println("#");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetLifeCycleData")) {
      Serial.println("Ok-GetLifeCycleData");
      Serial.flush();
    } else if (command.startsWith("GetWPSSensorData")) {
      Serial.println("Ok-GetWPSSensorData");
      Serial.flush();
    } else {
      //
      // call read to flush the incoming
      //
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
      delay(delayTime);
    }
  }
}

void animateLights(uint8_t r, uint8_t g,uint8_t b ){ 
    leds[4] = CRGB(r, g, b);
    leds[5] = CRGB(r, g, b);
    leds[6] = CRGB(r, g, b);
    leds[7] = CRGB(r, g, b);
    FastLED.show();    
}

void setApMode() {

  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  Serial.println("settting AP mode");
  //
  // set ap mode
  //
  //  wifiManager.configWifiAP("PanchoTankFlowV1", "", "PanchoTankFlowV1");
  String apAddress = wifiManager.getApAddress();
  Serial.println("settting AP mode, address ");
  Serial.println(apAddress);
  const uint8_t ap[] = {
    SEG_F | SEG_G | SEG_A | SEG_B | SEG_C | SEG_E,  // A
    SEG_F | SEG_G | SEG_A | SEG_B | SEG_E           // P
  };
  display1.setSegments(ap, 2, 0);
  delay(1000);
  uint8_t ipi;

  for (int i = 0; i < 4; i++) {
    ipi = GeneralFunctions::getValue(apAddress, '.', i).toInt();
    display1.showNumberDec(ipi, false);
    delay(1000);
  }
  for (int i = 2; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  leds[0] = CRGB(0, 255, 0);
  // if (loraActive) {
  //   leds[1] = CRGB(0, 0, 255);
  // } else {
  //   leds[1] = CRGB(255, 0, 0);
  // }

  FastLED.show();
}

void setStationMode(String ipAddress) {
  wifiActive = true;
  Serial.print("settting Station mode, address ");
  Serial.println(ipAddress);
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  const uint8_t ip[] = {
    SEG_F | SEG_E,                         // I
    SEG_F | SEG_G | SEG_A | SEG_B | SEG_E  // P
  };
  uint8_t ipi;
  for (int i = 0; i < 4; i++) {
    ipi = GeneralFunctions::getValue(ipAddress, '.', i).toInt();
    display1.showNumberDec(ipi, false);
    delay(1000);
  }
}
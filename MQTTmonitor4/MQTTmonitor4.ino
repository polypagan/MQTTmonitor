/********************************************
 * 
 * MQTTmonitor -- DdelV 20181104
 *                      20181108 status: working, ugly, but working.
 *                      20181124 not working well enough. Port to ESP8266
 *                      
 * First cut attempts to use (antique)
 * Adafruit CC3000 WiFi shield
 * with Uno.
 * I've added an I2C character display (20 chars, 4 rows)
 * {it would be nice to use ILI9341 TFT shield,
 *  but there likely isn't enough memory, and besides,
 *  likely pin conflicts.)
 * Both CC3000 & TFT shields have micro SDcard slots
 * (un-used so far).
 * 
 * Second cut:
 * ESP8266 Port -- use conditionals to get logic working, then backport -- works!
 * 
 * Third cut: (20181126)
 * Dispense with Finite State Machine approach.
 * Just make sure each level is connected.
 * 
 * Connect to WiFi network
 * Connect to MQTT broker
 * Subscribe to topics of interest
 * Display messages on screen.
 * 
 * Fourth cut: 20181226
 *  ESP8266, remove all AVR/CC3000 references
 *  use myLib/FSM
 *  working as of 20181227 -- (fix PubSubClient call and JsonBuffer dimensions)
 *  
 * 20190113 port to ESP32 WROOM32 w/64x128 OLED
 *  Status: working, a little messy.
 *  Program size exceeds 50% of ESP32 capacity, 
 *  so OTA can't work (OTA code not correct either).
 *  
 * 20190306 implemented SmartConfig in
 *  FSM.h & handleBTTN()
 */
#include <Arduino.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  #include <ESP8266HTTPUpdateServer.h>
#endif
#if defined (ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>    // there may be a better choice
  #include <ESPmDNS.h>
  #include <Update.h>
#endif

#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Time.h>
#include <Timezone.h>

#include <Wire.h>
#include <SPI.h>

#include <Streaming.h>          // write to Serial and LCD streams
#include <ArduinoJson.h>        // serialize/deserialize JSON messages
#include <PubSubClient.h>       // MQTT client

//#define I2C_LCD
//#define N5110 true
#define OLED64 true
//#define OLED128 true // if def, must define one of the following:
//#define WROOM32 true // "Wemos® ESP32 OLED Module WiFi + Bluetooth Dual ESP-32 ESP-32S" buttons on solder side
//#define FakeWeMOS // "Pro ESP32 OLED V2.0 TTGO" buttons on comp side

// the usual, ordinary blue LED on GPIO16 (D0), negative logic
#ifdef LED_BUILTIN
  #define LEDPIN LED_BUILTIN
#endif
#define LEDON  LOW
#define LEDOFF HIGH
  
#define hasBTTN false   // has flash button on GPIO0 (D3) used to adjust contrast
#define hasBUZZ true    // WeMOS buzzer shield jumpered to D8

const int16_t TVOC_LIMIT = 350; //ppb

#if hasBTTN           // nodeMCU with "Flash" button
  #include <EEPROM.h>
  #include <myESPEEPROM.h>
  #if defined(ESP8266)
    const byte BTTNPIN = D3;
  #endif
  #if defined(ESP32)
    const byte BTTNPIN = 0;
  #endif
  #define CONTRASTMIN 0x10  // lowest useful value I've found
  #define CONTRASTMAX 0x40  // highest ditto
  #define CONTRASTINC 0x02  // bump it by this much per BTTN press
  #define EEPROMSIZE 2
  #define CONTRAST_CELL 0
  #define DEFAULT_CONTRAST ((CONTRASTMAX - CONTRASTMIN)/2)
#endif    // hasBTTN
    
#if hasBUZZ
  const byte BUZZPIN = D8;
  const int16_t FREQ = 1047;
  const int16_t QUIET = 0;
  const int16_t BEEP = 512;
#endif

// 4x20 character LCD via I2C
#if defined(I2C_LCD)
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C  LCD(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

  #define MAXROW 3
  #define PIXperCHAR 1
#endif

// 48x84 graphics LCD via proprietary interface
#if defined(N5110)
  #include <Adafruit_GFX.h>
  #include <Adafruit_PCD8544.h>
  
  extern uint8_t TinyFont[];      // compact 4x8 font
  #if defined(ESP8266)    // Note: need modified library for ESP8266
    const byte RST = D6,  //MISO
               CS  = D8,  //SS
               DC  = D2,  //arbitrary
               DIN = D7,  //MOSI
               CLK = D5,  //SCK
               BL  = D4;  //TXD1
 
  #endif   // ESP8266
  
// add pin defs for other processors -- ATmega328, ESP32 etc.
  
  Adafruit_PCD8544 LCD = Adafruit_PCD8544(CLK, DIN, DC, CS, RST);

  #include <Fonts/TomThumb.h>
  #define H 5
  #define W 6
  #define bar true    // make a bar graph
  #define invert true // 0,0 in lower left
  #define BL true     // text origin is bottom left
  #include <prettyPlotPoint.h>
  #define MAXROW 7
  #define PIXperCHAR 6
  
  #define clear clearDisplay
 #endif // N5110

// 64x48 OLED via I2C 
#if defined(OLED64)
//  #include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <Fonts/TomThumb.h>

//  MicroOLED LCD(255, 0);
  Adafruit_SSD1306 LCD(64, 48, &Wire);

//  #define drawPixel(x,y,c) pixel(x,y)
//  #define drawLine(x1,y1,x2,y2,c) line(x1,y1,x2,y2)
//  #define clearDisplay() clear(PAGE)
  
//  #include <Fonts/TomThumb.h>
  #define MAXX 63
  #define MAXY 47
//  #define lastCol MAXX
  #define H 5
  #define W 6
  #define BL true
  #define bar true    // make a bar graph
  #define invert true // 0,0 in lower left
  #define BL false    // Text Origin is Top Left
  #include <prettyPlotPoint.h>
  #define MAXROW 7
  #define PIXperCHAR 6

#endif

// 128x64 OLED via I2C (2 versions)
#if defined(OLED128)
//  #include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
  #if defined(WROOM32)    // ESP32 WROOM32 OLED board
//    #include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    
    const byte OLED_SDA = 5;
    const byte OLED_SCK = 4;

    Adafruit_SSD1306 LCD(128, 64, &Wire); // 0x3c,OLED_SDA, OLED_SCK);
    
  #endif  // WROOM32
  #if defined(FakeWeMOS)     // Banggood fake WeMOS TTGO
//    #include <SSD1306.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    const uint8_t OLED_SDA = 4;
    const uint8_t OLED_SCK = 15;
    const uint8_t OLED_RST = 16;

    Adafruit_SSD1306 LCD(128, 64, &Wire, OLED_RST); // 0x3c,OLED_SDA, OLED_SCK);
    
    #define LEDPIN 2  // for now
    #define LEDON  HIGH
    #define LEDOFF LOW
  #endif  // FakeWeMOS
  
//  SSD1306Wire LCD(0x3c, OLED_SDA, OLED_SCK);
  
//  #define drawPixel(x,y,c) setPixel(x,y)
//  #define drawLine(x1,y1,x2,y2,c) drawLine(x1,y1,x2,y2)
//  #define clearDisplay() clear()

  #include <Fonts/TomThumb.h>
  #define MAXX 127
  #define MAXY 63
  #define bar true
  #define invert true
  #define BL true
  #define H 7
  #define W 6
  #include <prettyPlotPoint.h>
#endif  // OLED128

WiFiClient wifiClient;

unsigned long rightNow;
static uint8_t contrast;  // this is only used if N5110, needed for handleBTTN

#define WIN_301747
const char* server = "192.168.254.245";       // Onion Omega+ "C611" MQTT & NTP
#include "Net.h"
#include <NTP.h>

#if defined(I2C_LCD)
// trivial function to progress to next display line
  inline int8_t nxtrow(void) {static int8_t r=0; return r=(r>(MAXROW-1))?r=0:r+=PIXperCHAR; }
#endif

// topics to publish/subscribe to
const char* AIRQUALTOPIC =  "CC/grounds/coop/airquality"; // publish air quality readings
const char* ERRORTOPIC = "CC/ERRORS"; // something wrong
const char* ALARMTOPIC = "CC/ALARM";  // something VERY wrong, sound alarm
const char* willTopic = "CC/ERRORS";
const int willQoS = 0;
const boolean willRetain = false;
char willMsg[30];   // gets initialized in setup()

const uint16_t port=1883;

char clientID[22] = {"MQTTmonESP-"};

void buzz(void);

void callBack(char* topic, byte* message, uint16_t mlen) {
const int JBSIZE = JSON_OBJECT_SIZE(7);
//StaticJsonBuffer<JBSIZE> jsonBuffer;
StaticJsonBuffer<200> jsonBuffer;
char json[JBSIZE];
static uint16_t x = 0;  // for graphics plotting
int i;

  for (i=0; i<mlen; i++) {
//    LCD << message[i];
    json[i] = message[i];
  }
  json[i] = '\0';

//  LCD.setCursor(0,1*PIXperCHAR);
//  LCD << json;
  
//  Serial << "callBack(topic: " << topic << ", message: " << json << ", length: " << mlen << endl;
  
  JsonObject& root = jsonBuffer.parseObject(json);

  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }

 int16_t tvoc = root["TVOC"].as<int>();
 Serial << "TVOC:" << tvoc << endl;
 if(tvoc)x = plotNext(x,tvoc);    // don't bother plotting zeros
 LCD.display();

 if (tvoc > TVOC_LIMIT) {
  buzz();
 }
 
}

PubSubClient mqttClient(server, port, callBack, wifiClient);
#if defined(ESP32)
  WebServer httpServer(80);
#endif
#if defined(ESP8266)
  ESP8266WebServer httpServer(80);
  ESP8266HTTPUpdateServer httpUpdater;
#endif

// #define FSM_Serial 1
#include <FSM.h>    // Networking Finite State Machine (must be included after definition of client, clientID, and instantiation of server classes.)

/*
 * handleBTTN()
 * 
 * check for "flash" button pressed,
 * do primitive debounce,
 * increase LCD contrast in circular fashion.
 * 
 */
void handleBTTN(void) {
#if hasBTTN
int16_t d = 0;  // delay counter

  if (digitalRead(BTTNPIN) == LOW) { // button is pressed
    delay(50);
    while (digitalRead(BTTNPIN) == LOW) {  
      delay(1);   // wait for release
      if (++d > 6000) {     // if button is help for 6 seconds
        WiFi.disconnect(false);
        #if defined(LEDPIN)
          digitalWrite(LEDPIN, LEDOFF);
        #endif
        return;   // bail on contrast increment
      }
    }
    #if defined(N5110)
      contrast += CONTRASTINC;
      if (contrast > CONTRASTMAX)contrast = CONTRASTMIN;
      Serial << "contrast:" << contrast << endl;
      LCD.setContrast(contrast & 0x7f);
      sumChkEE(CONTRAST_CELL, contrast & 0xff);
    #endif
  }
#endif
}

void buzz(void) {
#if hasBUZZ
  analogWrite(BUZZPIN, BEEP);
  delay(100);
  analogWrite(BUZZPIN, QUIET);
#endif
}

void subscribeTopics(void ) {

  Serial << "subscribeTopics()" << endl;

// and subscribe to some topics    

      if (mqttClient.subscribe(AIRQUALTOPIC)) {
        Serial << "mqttClient.subscribe(" << AIRQUALTOPIC << ") ok." << endl;
        LCD << "Sub "; LCD.display();
      } else {
        Serial << "mqttClient.subscribe(" << AIRQUALTOPIC << ") failed." << endl;
      }
      if (mqttClient.subscribe(ALARMTOPIC)) {
        Serial << "mqttClient.subscribe(" << ALARMTOPIC << ") ok." << endl;
        LCD << "Sub "; LCD.display();
      } else {
        Serial << "mqttClient.subscribe(" << ALARMTOPIC << ") failed." << endl;
      }
      if (mqttClient.subscribe(ERRORTOPIC)) {
        Serial << "mqttClient.subscribe(" << ERRORTOPIC << ") ok." << endl;
        LCD << "Sub "; LCD.display();
      } else {
        Serial << "mqttClient.subscribe(" << ERRORTOPIC << ") failed." << endl;
      }

  Serial << "subs done." << endl;
}

void setup(void) {
// char ID[10];   // obsolete

  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nHello, MQTT!\n"); 

  #if defined(ESP32)
    uint64_t chipid = ESP.getEfuseMac();  // get chip MAC address
    ultoa((chipid & 0xffffff), willMsg, 16);  // mask it to six hex digits
    strcat(clientID, willMsg);                // append to clientID
    strcpy(willMsg, clientID);                // copy to willMsg
    strcat(willMsg, " Down!");                // make complete sentence
    
    Serial << "\n\n\nclientID:" << clientID << ", willMsg:" << willMsg << endl;
  #endif
  
  #if defined(ESP8266)
    ultoa(ESP.getChipId(), willMsg, 16);       // borrow willMsg string for a moment to hold chip ID
    strcat(clientID, willMsg);                 // append chipID to clientID
    Serial << "\n\nclientID:" << clientID << endl;
  
    strcpy(willMsg, clientID);                 // make strings equal, leaving clientID unchanged
  
    Serial << "clientID:" << clientID << ", willMsg:" << willMsg << endl;
    
    strcat(willMsg, " Down!");                 // append message to chipID
    Serial << "will message is " << willMsg << endl;
 
  #endif

  #if defined(LEDPIN)
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LEDOFF); // is this OFF?
  #endif
  
  connectionState = CHECK_CONNECTION;

#if hasBTTN
  pinMode(BTTNPIN, INPUT);  // _PULLUP);

  EEPROM.begin(EEPROMSIZE);
  if (EEchksum(CONTRAST_CELL) == 0) {
    contrast = getEEbyte(CONTRAST_CELL);
  } else {
    sumChkEE(CONTRAST_CELL, DEFAULT_CONTRAST);
    contrast = DEFAULT_CONTRAST;
  }
  Serial << "Has BTTN!" << endl;
#endif
  
#if defined (I2C_LCD)
  Serial << "Using I2C (char) LCD." << endl;
  Wire.begin();
  LCD.setBacklightPin(3, POSITIVE);
  LCD.begin(20,4);
  LCD.clear();
  LCD.backlight();
  LCD << "Hello, MQTT!      ";
#endif

#if defined (N5110)
  Serial << "Using Nokia LCD." << endl;
  LCD.begin();
  LCD.setContrast(contrast);
  LCD.setFont(&TomThumb);
  LCD.setTextSize(1);
  LCD.setTextColor(BLACK);
//  LCD.setFont(TinyFont);
#endif

#if defined(OLED64)
  Serial << "Using tiny OLED." << endl;
  Wire.begin();   // use system default SDA, SCK
//          Vcc source,           addr, reset,  begin  
  LCD.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false);
  LCD.setFont(&TomThumb);
  LCD.setTextColor(WHITE, BLACK);
  
//  LCD.clear(ALL);
//  LCD.display();
#endif

#if defined(OLED128)
  #if defined(FakeWeMOS)
    Wire.begin(OLED_SDA, OLED_SCK);
//                                  addr, reset, begin    
    LCD.begin(SSD1306_SWITCHCAPVCC, 0x3c, true, false);
    LCD.setFont(&TomThumb);
/* 
    pinMode(OLED_RST, OUTPUT); 
    digitalWrite(OLED_RST, LOW); 
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);
*/
    Serial << "Banggood fake WeMOS." << endl;
  #endif  // FakeWeMOS
  #if defined(WROOM32)
  Wire.begin(OLED_SDA, OLED_SCK);
  LCD.begin(SSD1306_SWITCHCAPVCC, 0x3c, false,false);
  LCD.setFont(&TomThumb);
  #endif  // WROOM32

//  LCD.init();
//  LCD.flipScreenVertically();
//  LCD.setFont(ArialMT_Plain_10);  
//  LCD.display();
  LCD.setTextSize(1);
  LCD.setTextColor(WHITE, BLACK);
#endif

  Serial << "survived LCD setup." << endl;
  
#if hasBUZZ
  Serial << "Has BUZZ!" << endl;
  
  pinMode(BUZZPIN, OUTPUT);
  digitalWrite(BUZZPIN, 0);
  analogWriteRange(FREQ);

  buzz();
#endif
/*
  Serial << "ready to clear?";
  LCD.clearDisplay();
  Serial << "...cleared!" << endl;
*/  
/*****
#if defined(ESP32)
// following chunk probably (???) needs to be in FSM.h
   // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with its md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
// end chunk.
#endif
*****/

  rightNow = millis();

  Serial << "void setup(void ) done!" << endl;
  
}   // end setup


void loop(void) {
  
  do {
        FSM();
        mqttClient.loop();                // Run MQTT client loop code
//        httpServer.handleClient();    // Run http client loop for OTA update 
        ArduinoOTA.handle();
        handleBTTN();     // this can preempt, but is important to pause and get contrast +/- correct.
  } while ((millis() - rightNow) < 1000);

  rightNow = millis();
}

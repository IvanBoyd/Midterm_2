/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/boyd/Documents/IoT/Midterm_2/MidTerm2/src/MidTerm2.ino"
/*
 * Project MidTerm2
 * Description: 
 * Started:
 *  2. Publish soil moisture & room env data to new dashboard
 * To Do: 
 *  3. Auto water plant when soil is too dry (pump only ~1/2 second)
 *  4. Int a button on dashboard that manually waters the plant
 *  Done:   
 *    1. Int 2N3906 Emitter Follower & Relay & BME w Disp
 * Author:      Ivan Boyd
 * Date:        11/08/21
 * History: <-L14_03_SubscribePublish.ino <-L14_02_v2_Moisture.ino
 */

// setup() runs once, when the device is first turned on.
#include <Adafruit_BME280.h>                // temp, pressure & humidity
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"  
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "credentials.h"

/************ Global State (you don't need to change this!) ***   ***************/ 
void setup();
void loop();
void runBMEchk();
float CtoF(float _tempC);
void MQTT_connect();
#line 25 "c:/Users/boyd/Documents/IoT/Midterm_2/MidTerm2/src/MidTerm2.ino"
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqttObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/FeedNameA"); //Room Temp
Adafruit_MQTT_Publish mqttObj3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/FeedSoilMoist");

Adafruit_MQTT_Subscribe mqttObj2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/FeedNameB");

/************Declare Variables*************/
unsigned long last, lastTime;
float value1, value2;
int   MQTTbuttVal;
const int LED_PIN = 9,
          D7_LED  = D7;       // not needed just call D7 directly
// int   rando = 0,  lastSec = 0, currentTime = 0;

SYSTEM_MODE(SEMI_AUTOMATIC);

             // OLED display
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);

const int SOIL_MOIST_PIN = 19;
int soilMoistVal = 0;

// Set up Adafruit_BME280 Environment for temp, pressure & humidity
Adafruit_BME280 bme;      // for the I2C device
float tempC,          tempF,        tempinHg,
      pressPA,        humidRH,      fiveMinAvg;
bool  bmeStatus;
byte  hexAddress=0x76; 

String DateTime, TimeOnly;

// SYSTEM_MODE(AUTOMATIC);          // Default if no SYSTEM_MODE included
// SYSTEM_MODE(SEMI_AUTOMATIC);     // Uncomment if using without Wifi
// SYSTEMF_MODE(MANUAL);            // Fully Manual
void setup() {
    pinMode(SOIL_MOIST_PIN,INPUT);
    Time.zone(-7);          //MST = -7, MDT = -6
    Particle.syncTime();    // Sync time with Particle Cloud
    Serial.begin(9600);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    display.display();                          // show splashscreen on 1306 OLED
    delay(2000);
    display.clearDisplay();   // clears the OLED screen and buffer

    // draw a single pixel
    display.drawPixel(10, 10, WHITE);
    display.display();
    delay(2000);
    display.clearDisplay();
    runBMEchk();        // check temp module BME280
    waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup

    //Connect to WiFi without going to Particle Cloud
    WiFi.connect();
    while(WiFi.connecting()) {              // Will breath green when connected
      Serial.printf(".");                   // if don't neet time but need the net then can do this
    }

    // Setup MQTT subscription for onoff feed.
    mqtt.subscribe(&mqttObj2);
    // rando = random();
    pinMode(LED_PIN,OUTPUT);
    pinMode(D7,OUTPUT);
    tempF = CtoF(bme.readTemperature());
    soilMoistVal = analogRead(SOIL_MOIST_PIN);   
}                   //   ***  END OF SETUP  ***

void loop() {
  // New Publish Code
  // Validate connected to MQTT Broker
  // currentTime = millis();
  MQTT_connect();
  //  if((currentTime - lastSec)>6000) {
  //    Serial.printf("six seconds have passed /n");
  //  }
  //  lastSec = millis();
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }

  // publish to cloud every 30 seconds
  value1 = random(0,100);
  if((millis()-lastTime > 30000)) {
    if(mqtt.Update()) {
      mqttObj1.publish(tempF);
      mqttObj3.publish(soilMoistVal);

      Serial.printf("Publishing %0.2f  TempF: %0.2f\n",value1, tempF); 
      } 
    lastTime = millis();
  }


  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &mqttObj2) {
      value2 = atof((char *)mqttObj2.lastread);
          Serial.printf("Received %0.2f from Adafruit.io feed FeedNameB \n",value2);
      MQTTbuttVal = atoi((char *)mqttObj2.lastread);
      if(MQTTbuttVal == 1)  {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(D7, HIGH);

        Serial.printf("MQTTbuttVal = %i converted w atoi of value2 (%0.2f) feed FeedNameB \n",MQTTbuttVal, value2);

      }
       if(MQTTbuttVal == 0)  {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(D7, LOW);
        Serial.printf("MQTTbuttVal = %i converted w atoi of value2 (%0.2f) feed FeedNameB \n",MQTTbuttVal, value2);

      }     
    }
  }     // End While
// End NEW Publish code

   soilMoistVal = analogRead(SOIL_MOIST_PIN);
   tempF = CtoF(bme.readTemperature());
   Serial.printf("Moisture reg in pin: %i, Temp: %f\n", soilMoistVal, tempF);
   delay(3000);
      // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Trancer DEER");

  display.display();
//   delay(6000);
  display.clearDisplay();

  DateTime = Time.timeStr();                      //Current Date & Time from Particle Time
  TimeOnly = DateTime.substring(11,19);           //Ext time from datetime str
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(3,0);
  display.printf("Date/Time:%s\n",DateTime.c_str());
  display.printf("Time: %s\n",TimeOnly.c_str());
  display.printf("Temp: %f\n", tempF);

  display.display();
  Serial.printf("Date & Time is %s\n",DateTime.c_str());
  Serial.printf("Time is %s\n",TimeOnly.c_str());
  delay(1000);
}                     //  *** END OF MAIN VOID LOOP ***

                //      ***    F U N C T I O N S    ***

void runBMEchk()  {         // check status of BME/temp and send to print
  bmeStatus  = bme.begin(hexAddress);
  if(bmeStatus ==  false)  {
    Serial.printf("BME280 at address 0x%02X failed to start\n", hexAddress);
    delay(4000);
   }
   else {
    tempC   = bme.readTemperature();  //deg C
    tempF = CtoF(tempC);
    Serial.printf("Temperature at startup: %f \n",tempF);
    delay(2000);
   }
}

float CtoF(float _tempC) {     //Convert Celcius to Fahrenheit °F = (°C × 9/5) + 32//
  float _tempF;
  _tempF = (_tempC * 9/5) +32;
  return _tempF;
}

// Two new functions that will be useful :
// atof () - ASCII to Float : converts an ASCII string to a floating point number
// atoi () - ASCII to Integer : converts an ASCII string to an integer

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}
/*
 * Project MidTerm2
 * Description: 
 * Started:
 
 *  Done:   
 *    1. Int 2N3906 Emitter Follower & Relay & BME w Disp
 *  * 2. Publish soil moisture & room env data to new dashboard
 *    3. Auto water plant when soil is too dry (pump only ~1/2 second)
 *    4. Int a button on dashboard that manually waters the plant
 * Author:      Ivan Boyd
 * Date:        11/08/21
 * History: <-L14_03_SubscribePublish.ino <-L14_02_v2_Moisture.ino
 */

// setup() runs once, when the device is first turned on.
#include <Adafruit_BME280.h>                // temp, pressure & humidity
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"  
#include "Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT.h"
#include "credentials.h"


/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqttObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/FeedNameA");             //Room Temp
Adafruit_MQTT_Publish mqttObj3 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/FeedSoilMoist");

Adafruit_MQTT_Subscribe mqttObj2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/FeedNameB");         // receives button press

/************Declare Variables*************/
unsigned long last, lastTime;
float value1, value2;
int   MQTTbuttVal;
const int LED_PIN = 9,
          D7_LED  = D7,               // not needed just call D7 directly
          PUMP_RELAY_PIN  = D12;       // this is pin M0 or MO
bool  runPump = true;                 // !!!  must be set true when watering !!!
// bool  runPump = false;   
bool      deBug  = false;            // !!! set false when when not testing !!!

SYSTEM_MODE(SEMI_AUTOMATIC);

             // OLED display
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);

const int SOIL_MOIST_PIN    = 19;
const int TIME2WATER_PLANT  = 3100;         // Set this value based on, Pure Air = 3,500, Pure Water = 1,700, Reading in Dry Soil = 3,280
int           soilMoistVal  = 0;

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
      Serial.begin(9600);

    pinMode(SOIL_MOIST_PIN,INPUT);
    pinMode(PUMP_RELAY_PIN,OUTPUT);
    waterPlantHalfSec();
//     // if(runPump==true) {
//            Serial.printf("test if setup & run pump works /n");
// delay(4000);
//       digitalWrite(PUMP_RELAY_PIN,HIGH);
//       delay(3000);
//       digitalWrite(PUMP_RELAY_PIN,LOW);
//             delay(3000);

    // }

    Time.zone(-7);          //MST = -7, MDT = -6
    Particle.syncTime();    // Sync time with Particle Cloud
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    display.display();                          // show splashscreen on 1306 OLED
    delay(2000);
    display.clearDisplay();   // clears the OLED screen and buffer

    // draw a single pixel
    display.drawPixel(10, 10, WHITE);
      display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0,0);
      display.println("Trancer DEER");
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
    // if(runPump==true) {
    //   digitalWrite(PUMP_RELAY_PIN,HIGH);
    //   delay(2000);
    //   digitalWrite(PUMP_RELAY_PIN,LOW);
    //         delay(3000);
    // }
    // }  // New Publish Code
    waterPlantTillMoist();
    // delay(3000);

    //  if(runPump==true && soilMoistVal > 2400) {
    //     while(soilMoistVal > 2400) {
    //       waterPlantHalfSec();
    //       soilMoistVal = analogRead(SOIL_MOIST_PIN); 

    //     }
    //  }
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
        // digitalWrite(LED_PIN, HIGH);
        digitalWrite(D7, HIGH);
        waterPlantHalfSec();

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
  display.printf("SoilMoist: %i\n", soilMoistVal);
  display.display();
  Serial.printf("Date & Time is %s\n",DateTime.c_str());
  Serial.printf("Time is %s\n",TimeOnly.c_str());
  delay(1000);
}                     //  *** END OF MAIN VOID LOOP ***



                //      ***    F U N C T I O N S    ***

void waterPlantTillMoist()  {
int _currTime = millis(), _newtime = millis();
    if(deBug) { 
      _newtime = millis();
      Serial.printf("deBug: Watering the plant till Moist: %i runPump: %i), millis: %i, _currTime: %i\n", soilMoistVal, runPump, _newtime, _currTime);
      delay(1000);
    }
    if(runPump && soilMoistVal > TIME2WATER_PLANT) {        // high soilMoistVal == dryer soil
    while(soilMoistVal > TIME2WATER_PLANT) {
      waterPlantHalfSec(); 
       _newtime = millis();
 
      Serial.printf("Watering the plant till Moist: %i runPump: %i), millis: %i, _currTime: %i\n", soilMoistVal, runPump, _newtime, _currTime);

                          // for a half sec
      while((millis() - _currTime) < 1000)   {
      }            // after plant has rcvd 1/2 sec water, then wait 1 sec bef giving more
      soilMoistVal = analogRead(SOIL_MOIST_PIN); 
    }
    Serial.printf("Finished watering plant: %i runPump: %i), millis: %i, _currTime: %i\n", soilMoistVal, runPump, _newtime, _currTime);


  }
}

void waterPlantHalfSec()  {                   // waters plant for 1/2 second
int _currTime = millis();
  while((millis() - _currTime) < 500) {       // water until 1/2 sec has passed
    if(deBug==true) { 
      Serial.printf("runPump: (%i), millis: %i, _currTime: %i\n", runPump, millis(), _currTime);
      delay(1000);
    }
    if(runPump) {
      digitalWrite(PUMP_RELAY_PIN,HIGH);
      }
    else {
      Serial.printf("runPump is false (%i), imagine plant being watered *ON*\n", runPump);
      delay(2000);
      }
  }
  if(runPump) {                         // pump has run 1/2 second so turn it off
    digitalWrite(PUMP_RELAY_PIN,LOW);
  }
  else {
    Serial.printf("runPump is false (%i), imagine pump is now #OFF# plant being watered\n", runPump);
    delay(2000);
  } 
}


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
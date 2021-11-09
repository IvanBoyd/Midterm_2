/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/boyd/Documents/IoT/Midterm_2/MidTerm2/src/MidTerm2.ino"
/*
 * Project MidTerm2
 * Description: 
 *  1. Int 2N3906 Emitter Follower & Relay & BME w Disp
 *  2. Publish soil moisture & room env data to new dashboard
 *  3. Auto water plant when soil is too dry (pump only ~1/2 second)
 *  4. Int a button on dashboard that manually waters the plant
 * Author:      Ivan Boyd
 * Date:        11/08/21
 * History: <-L14_02_v2_Moisture.ino
 */

// setup() runs once, when the device is first turned on.
#include <Adafruit_BME280.h>                // temp, pressure & humidity
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"               // OLED display
void setup();
void loop();
void runBMEchk();
float CtoF(float _tempC);
#line 17 "c:/Users/boyd/Documents/IoT/Midterm_2/MidTerm2/src/MidTerm2.ino"
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
}                   //   ***  END OF SETUP  ***

void loop() {

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

/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/pjb/Dropbox/smart-coasts-bathy-mapping/Firmware/depth-sensing-ping-SD/src/depth-sensing-ping-SD.ino"
/*
 * JSN-SR04T Ultrasonic Range Finder Ping Mode Test
 *
 * Exercises the ultrasonic range finder module and prints out measured distance
 * 5V - connect to 5V
 * GND - connect to ground
 * RX/TRIG - connect to digital pin 12.  Can be any digital pin
 * TX/ECHO - connect to digital pin 13.  Can be any digital pin
 */
void setup();
void loop();
#line 10 "/Users/pjb/Dropbox/smart-coasts-bathy-mapping/Firmware/depth-sensing-ping-SD/src/depth-sensing-ping-SD.ino"
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;
float temp_In_C = 20.0;  // Can enter actual air temp here for maximum accuracy
float speed_Of_Sound;          // Calculated speed of sound based on air temp
float distance_per_usec;      // Distance sound travels in one microsecond

long real_time;
int millis_now;

//------------------SD SPI Configuration Details--------------------------------
#include "SdFat.h"
const int SD_CHIP_SELECT = D5;
SdFat sd;
File myFile;

//------------------LED Setup
int led = D7; // blink to let us know you're alive
bool led_state = HIGH; // starting state

// Global objects
FuelGauge batteryMonitor;

// To use Particle devices without cloud connectivity
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

//===============================================================================
//  Initialization
//=============================================================================== 
void setup() {

  // Set up trigger and echo pins
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  // TODO: PJB to update sound speed calc using TEOS-10 and temp/salinity/pressure measurement
  // Formula to calculate speed of sound IN AIR in meters/sec based on temp
  speed_Of_Sound = 331.1 +(0.606 * temp_In_C); 

  // Calculate the distance that sound travels in one microsecond in centimeters
  distance_per_usec = speed_Of_Sound / 10000.0;

  Serial.begin(9600);

  // Initialize the SD library
  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
    Serial.println("failed to open card");
    return;
  }

  // Start SD stuff
  // open the file for write at end like the "Native SD library"
  if (!myFile.open("depth.csv", O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println("opening test.txt for write failed");
    return;
  }

  // Save column headers to SD card
  myFile.println("real_time,millis,depth (cm)");
  myFile.close();

}

//===============================================================================
//  Main
//===============================================================================
void loop() {
  float duration, depth_cm;

  // "Real" time and current millis for logging
  real_time = Time.now();
  millis_now = millis();
 
  digitalWrite(TRIG_PIN, HIGH);       // Set trigger pin HIGH 
  delayMicroseconds(20);              // Hold pin HIGH for 20 uSec
  digitalWrite(TRIG_PIN, LOW);        // Return trigger pin back to LOW again.
  duration = pulseIn(ECHO_PIN,HIGH);  // Measure time in uSec for echo to come back.
 
  // convert the time data into a distance in centimeters
  depth_cm = duration/2 * distance_per_usec;
   
  // Print results, good or bad
  if (depth_cm <= 0){
    // bad values
    Serial.println("Out of range"); 
  }
  else {
    // good values
    Serial.print(real_time);
    Serial.print(",");
    Serial.print(millis_now);
    Serial.print(",");
    Serial.println(depth_cm);
  }

  // open the file for write at end like the "Native SD library"
  if (!myFile.open("depth.csv", O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println("opening test.txt for write failed");
    return;
  }

  // Save to SD card
  myFile.print(real_time);
  myFile.print(",");
  myFile.print(millis_now);
  myFile.print(",");
  myFile.println(depth_cm);
  myFile.close();

  // Delay between readings
  delay(950);
}
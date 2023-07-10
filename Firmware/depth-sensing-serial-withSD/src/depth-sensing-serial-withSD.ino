/*
 * JSN_SR04T Ver 3.0 Ultrasonic Test - Serial Interface Mode
 * from https://protosupplies.com/product/jsn-sr04t-v3-0-waterproof-ultrasonic-range-finder/
 * adapted by PJB 7-July-2023. PJB's is intended for Particle HW without software serial. See 
 * website above for an Arduino Uno/SoftwareSerial version
 *
 * MODE 2: MCU Controlled Serial Mode
 * To enter Mode 2, the M2 pads are shorted OR a resistor value of 120K is placed across the MODE pads.
 * This mode is the same as Mode 1 except that the module takes a measurement only when a trigger command of 0x55 is sent to the RX pin on the module. The module then takes a single measurement and outputs the distance on its TX pin.
 * The minimum trigger cycle time is 60mS
 * 
 * Exercises the ultrasonic range finder module in serial mode 
 * and print out the current measured distance
 * VCC - connect to 3V3 
 * GND - connect to GND
 * Serial pins are cross connected RX to TX and TX to RX
 * TRIG/TX pin - connect to Particle RX pin
 * ECHO/RX - connect to Particle TX pin
 * 
 * For Serial mode of operation, ensure the M2 location is shorted
 */

#include "SdFat.h"
 //------------------SD SPI Configuration Details--------------------------------
const int SD_CHIP_SELECT = D5;
SdFat sd;

//------------------LED Setup
int led = D7; // blink to let us know you're alive
bool led_state = HIGH; // starting state

//------------------JSN Sensor 
byte StartByte = 0; // 0xFF signifies a new packet of info coming in
byte MSByte = 0; // Most Significant Byte of distance measurement
byte LSByte = 0; // Least Significant Byte of distance measurement
byte CheckSum = 0; // CheckSum = 0xFF + MSByte + LSByte
unsigned int mmDist = 0; // Returned distance in millimeters

//------ Leftover from water level
int j;
float dist_in_sum;
float dist_in_avg;
float range_cm;

long real_time;
int millis_now;

float filterArray[200]; // array to store data samples from sensor
float distance_unconverted; // store the distance from sensor

//------------------State variables
// not yet used but placeholders in case of additional states
// enum State {
//   DATALOG_STATE,
//   PUBLISH_STATE,
//   SLEEP_STATE
// };
// State state = DATALOG_STATE;
unsigned long stateTime = 0;

//------------------Turn off cellular for prelim testing; turn on for deployment
//SYSTEM_MODE(MANUAL); // uncomment for prelim testing
SYSTEM_MODE(SEMI_AUTOMATIC); // uncomment for deployment
SYSTEM_THREAD(ENABLED);

// Global objects
FuelGauge batteryMonitor;
//PMIC pmic;
const char * eventName = "waterDepth";

SystemSleepConfiguration config;

// Various timing constants
const unsigned long MAX_TIME_TO_PUBLISH_MS = 60000; // Only stay awake for 60 seconds trying to connect to the cloud and publish
const unsigned long TIME_AFTER_PUBLISH_MS = 4000; // After publish, wait 4 seconds for data to go out

void setup(void) {
  // Particle.connect();
  // Cellular.off(); // turn off cellular for prelim testing (uncomment for Boron, leave commented out for Argon)

  delay(5000); // to see response from begin command

  // Initialize the library
  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
    Serial.println("failed to open card");
    return;
  }

  Serial.begin(9600);
  Serial.println("JSN Test");
  Serial1.begin(9600); // Initialize the software serial port

}

void loop(void) {

  Serial1.flush(); // Clear the serial port buffer   
  Serial1.write(0x55); // Request distance measurement
  delay(100); // Give sensor time to make the measurement (>60mSec)

  // Looking for 4 bytes to be returned
  while(!Serial1.available() >= 4) { 
    delay(100); // wait for 4 bytes
  } 

  // Interpret incoming data
  StartByte = Serial1.read(); // Read first byte 
  
  if (StartByte == 0xFF) { // 1st byte is 0xFF for new measurement
    MSByte = Serial1.read(); // Read in the MSB (Most Significant Byte)
    LSByte = Serial1.read(); // Read in the LSB (Least Significant Byte)
    CheckSum = Serial1.read(); // Read the checksum byte
    mmDist = MSByte * 256 + LSByte; // Calculate the distance from the two bytes
  } else {
    Serial1.flush(); // Flush the buffer until valid start byte
  }

  // Get all metrics which are to be reused:
  // Convert analog signal to centimeters
  // range_cm = (float)distance_unconverted * 0.25; // conversion factor for MB7092 XL-MaxSonar-WRMA1; TODO: check new sensor's datasheet

  // "Real" time and current millis for logging
  real_time = Time.now();
  millis_now = millis();

  // Print out distance
  Serial.print(real_time);
  Serial.print(",");
  Serial.print(millis_now);
  Serial.print(",");
  Serial.println(mmDist);

  // Start SD stuff
  File myFile;

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
  myFile.println(mmDist);
  myFile.close();

  delay(900);
  //}


}
/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/pjb/Dropbox/Smart_Coasts_Sensors/smart-coasts-bathy-mapping/Firmware/depth-sensing-ping-SD-GPS-temp/src/depth-sensing-ping-SD-GPS-temp.ino"
/*
* JSN-SR04T Ultrasonic Range Finder Ping Mode Test
*
* Exercises the ultrasonic range finder module and prints out measured distance
* 5V - connect to 5V
* GND - connect to ground
* RX/TRIG - connect to digital pin 12. Can be any digital pin
* TX/ECHO - connect to digital pin 13. Can be any digital pin
*/


void setup();
void loop();
void printToFile();
void serialPrintGPSTime();
void serialPrintGPSLoc();
void publishData();
#line 12 "/Users/pjb/Dropbox/Smart_Coasts_Sensors/smart-coasts-bathy-mapping/Firmware/depth-sensing-ping-SD-GPS-temp/src/depth-sensing-ping-SD-GPS-temp.ino"
const int TRIG_PIN = A2;
const int ECHO_PIN = A1;
float speed_Of_Sound; // Calculated speed of sound based on air temp
float distance_per_usec; // Distance sound travels in one microsecond


long real_time;
int millis_now;



// ------------------ SD SPI Configuration Details --------------------------------
#include "SdFat.h"
const int SD_CHIP_SELECT = D5;
SdFat SD;

char filename[] = "YYMMDD00.csv"; // template filename (year, month, day, 00–99 file number for that day)
bool filenameCreated = false;

// ---------------------------- GPS ----------------------------------------------
#include <Adafruit_GPS.h>
#define GPSSerial Serial1
Adafruit_GPS GPS( & GPSSerial);
uint32_t timer = millis();



//-------------------------- LED Setup -------------------------------------------
const pin_t MY_LED = D7; // blink to let us know you're alive
bool led_state = HIGH; // starting state


// Global objects; TODO: save power stats
FuelGauge batteryMonitor;


// To use Particle devices without cloud connectivity
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

unsigned long lastPublishTime = 0;  // Track when we last sent data
const unsigned long publishInterval = 300000;  // 5 minutes (in milliseconds)


//===============================================================================
// INITIALIZATION
//===============================================================================
void setup() {

//  Particle.connect();

    pinMode(MY_LED, OUTPUT);

    // Set up trigger and echo pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);


    //Serial.begin(9600);
    Serial.begin(115200);
    Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

    // Set up GPS
    GPS.begin(9600);

    // uncomment next line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // uncomment next line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz
    

    // Initialize the SD library
    if (!SD.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
        Serial.println("failed to open card");
        return;
    }

}


//===============================================================================
// MAIN
//===============================================================================
void loop() {

    

    // read data from the GPS in the 'main loop'
    char c = GPS.read();

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
    }

    // approximately every second or so, print out the current stats
    if (millis() - timer > 1000) {
        timer = millis(); // reset the timer


        serialPrintGPSTime();


        // If GPS gets a fix, print out and save good data
        if (GPS.fix) {  
            // Blink to let us know you're alive
            led_state = !led_state;
            digitalWrite(MY_LED, led_state); // turn the LED on (HIGH is the voltage level)

            serialPrintGPSLoc();

        }

        printToFile();

    }
     // Get battery charge if Boron provides it
    float cellVoltage = batteryMonitor.getVCell();
    float stateOfCharge = batteryMonitor.getSoC();

Serial.println(GPS.lon);

if (millis() - timer > 300000) {
publishData();
}
}

/* ---------------------- PRINT TO SD CARD FUNCTION ---------------------- */
void printToFile() {

  if (!filenameCreated) {
    // Get year, month, and day for filename
    int filenum = 0; // start at zero and increment by one if file exists
    sprintf(filename, "%02d%02d%02d%02d.csv", GPS.year, GPS.month, GPS.day, filenum);

    // Check for existence of filename with current filenum
    while (SD.exists(filename)) {
      filenum++;
      sprintf(filename, "%02d%02d%02d%02d.csv", GPS.year, GPS.month, GPS.day, filenum);
    }
    filenameCreated = true;
  }

  Serial.println(filename);

  // Create filename
  // Open the file: SPI SD comms
  File dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    // Date
    dataFile.print(GPS.month, DEC);
    dataFile.print('/');
    dataFile.print(GPS.day, DEC);
    dataFile.print("/20");
    dataFile.print(GPS.year, DEC);
    dataFile.print(",");

    // Time
    dataFile.print(GPS.hour, DEC);
    dataFile.print(':');
    if (GPS.minute < 10) {
      dataFile.print('0');
    }
    dataFile.print(GPS.minute, DEC);
    dataFile.print(':');
    if (GPS.seconds < 10) {
      dataFile.print('0');
    }
    dataFile.print(GPS.seconds, DEC);
    dataFile.print(".");
    if (GPS.milliseconds < 10) {
      dataFile.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      dataFile.print("0");
    }
    dataFile.println(GPS.milliseconds);
    dataFile.print(",");

    // Elapsed Time
    dataFile.print(millis() / 1000);
    dataFile.print(",");

    // Location
    dataFile.print(GPS.latitude, 4);
    dataFile.print(",");
    dataFile.print(GPS.lat); // N or S
    dataFile.print(",");
    dataFile.print(GPS.longitude, 4);
    dataFile.print(",");
    dataFile.print(GPS.lon); // E or W
    dataFile.print(",");

  

    //Altitude
    dataFile.print(GPS.altitude);
    dataFile.print(",");

    dataFile.print(GPS.speed);
    dataFile.print(",");

    // Angle
    dataFile.println(GPS.angle);

    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void serialPrintGPSTime() {
  Serial.print("\nTime: ");
  
  // Hour
  if (GPS.hour < 10) {
    Serial.print('0');
  }
  Serial.print(GPS.hour, DEC);

  Serial.print(':');

  // Minute
  if (GPS.minute < 10) {
    Serial.print('0');
  }
  Serial.print(GPS.minute, DEC);

  Serial.print(':');

  // Seconds
  if (GPS.seconds < 10) {
    Serial.print('0');
  }
  Serial.print(GPS.seconds, DEC);
  Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  Serial.println(GPS.milliseconds);
  
  Serial.print("Date: ");
  Serial.print(GPS.month, DEC);
  Serial.print('/');
  Serial.print(GPS.day, DEC);
  Serial.print("/20");
  Serial.println(GPS.year, DEC);
  
  Serial.print("Fix: ");
  Serial.print((int) GPS.fix);
  
  Serial.print(" quality: ");
  Serial.println((int) GPS.fixquality);
}

void serialPrintGPSLoc() {
  Serial.print("Location: ");
  Serial.print(GPS.latitude, 4);
  Serial.print(GPS.lat);
  Serial.print(", ");
  Serial.print(GPS.longitude, 4);
  Serial.println(GPS.lon);
}



void publishData() {
    // Format the data as a string with sensor readings
    String data = String::format(
        "Time: %d, GPS_Lat: %li, GPS_Lon: %li",
        Time.now(), GPS.latitude, GPS.longitude
    );

    // Publish the data to the Particle Cloud
    Particle.publish("sensor-data", data, PRIVATE);

    // Log the time of this publish event
    lastPublishTime = millis();
    
    // Print the data to the serial monitor for debugging
    Serial.println("Published Data: " + data);
}



// /* ---------------------- PUBLISH DATA FUNCTION? ---------------------- */

// // Prep for cellular transmission
//     bool isMaxTime = false;
//     stateTime = millis();

//     while (!isMaxTime) {
//       //connect particle to the cloud
//       if (Particle.connected() == false) {
//         Particle.connect();
//         Log.info("Trying to connect");
//       }

//       // If connected, publish data buffer
//       if (Particle.connected()) {

//         Log.info("publishing data");

//         // bool (or Future) below requires acknowledgment to proceed
//         bool success = Particle.publish(GPS.lat, GPS.lon);
//         Log.info("publish result %d", success); 

//         isMaxTime = true;
//         state = SLEEP_STATE;
//       }
//       // If not connected after certain amount of time, go to sleep to save battery
//       else {
//         // Took too long to publish, just go to sleep
//         if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
//           isMaxTime = true;
//           state = SLEEP_STATE;
//           Log.info("max time for publishing reached without success; go to sleep");
//         }
//         Log.info("Not max time, try again to connect and publish");
//         delay(500);
//       }
//     }
//   }
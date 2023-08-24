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

/* TODO: 
* make temperature global to access across functions
* log temperature
* 
*/

void setup();
void loop();
float getDepth();
float getTemp();
#line 17 "/Users/pjb/Dropbox/Smart_Coasts_Sensors/smart-coasts-bathy-mapping/Firmware/depth-sensing-ping-SD-GPS-temp/src/depth-sensing-ping-SD-GPS-temp.ino"
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

// ------------------------ Temperature ------------------------------------------
#include <Adafruit_MAX31865.h>
const int TEMP_CHIP_SELECT = D6;
Adafruit_MAX31865 sensor = Adafruit_MAX31865(TEMP_CHIP_SELECT);
#define RREF 430.0 //Rref resistor value = 430.0


//-------------------------- LED Setup -------------------------------------------
const pin_t MY_LED = D7; // blink to let us know you're alive
bool led_state = HIGH; // starting state


// Global objects
FuelGauge batteryMonitor;


// To use Particle devices without cloud connectivity
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);


//===============================================================================
// INITIALIZATION
//===============================================================================
void setup() {

    pinMode(MY_LED, OUTPUT);

    // Set up trigger and echo pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);


    //Serial.begin(9600);
    Serial.begin(115200);
    Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

    sensor.begin(MAX31865_3WIRE);

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


        Serial.print("\nTime: ");
        if (GPS.hour < 10) {
            Serial.print('0');
        }
        Serial.print(GPS.hour, DEC);
        Serial.print(':');
        if (GPS.minute < 10) {
            Serial.print('0');
        }
        Serial.print(GPS.minute, DEC);
        Serial.print(':');
        if (GPS.seconds < 10) {
            Serial.print('0');
        }
        Serial.print(GPS.seconds, DEC);
        Serial.print('.');
        if (GPS.milliseconds < 10) {
            Serial.print("00");
        }
        else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
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

        getDepth();


        // If GPS gets a fix, print out and save good data
        if (GPS.fix) {  
            // Blink to let us know you're alive
            led_state = !led_state;
            digitalWrite(MY_LED, led_state); // turn the LED on (HIGH is the voltage level)


            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4);
            Serial.print(GPS.lat); // N or S
            Serial.print(", ");
            Serial.print(GPS.longitude, 4);
            Serial.println(GPS.lon); // E or W
            Serial.print("Altitude: ");
            Serial.println(GPS.altitude);
            Serial.print("Speed: ");
            Serial.print(GPS.speed);
            Serial.print("Angle: ");
            Serial.print(GPS.angle);


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


                // Depth/distance
                dataFile.print(getDepth());
                dataFile.print(",");


                //Altitude
                dataFile.print(GPS.altitude);
                dataFile.print(",");


                dataFile.print(GPS.speed);
                dataFile.print(",");


                //dataFile.println(GPS.angle);
                dataFile.println(GPS.angle);


                dataFile.close();
            }
            // if the file isn't open, pop up an error:
            else {
                Serial.println("error opening datalog.txt");
            }

        }

    }

}



/* ---------------------- GET DEPTH FUNCTION ---------------------- */
float getDepth() {
    float duration, depth_cm;

    // Send pulse and listen to response
    digitalWrite(TRIG_PIN, HIGH); // Set trigger pin HIGH
    delayMicroseconds(20); // Hold pin HIGH for 20 uSec
    digitalWrite(TRIG_PIN, LOW); // Return trigger pin back to LOW again.
    duration = pulseIn(ECHO_PIN, HIGH); // Measure time in uSec for echo to come back.


    speed_Of_Sound = 331.1 + (0.606 * getTemp());
    // Calculate the distance that sound travels in one microsecond in centimeters
    distance_per_usec = speed_Of_Sound / 10000.0;

    // convert the time data into a distance in centimeters
    depth_cm = duration / 2 * distance_per_usec;


    if (depth_cm <= 0) {
        // bad values
        Serial.println("Out of range");
    }
    else {
        // good values
        Serial.print(real_time);
        Serial.print(",");
        Serial.print(millis());
    }
    return depth_cm;


}


/* ---------------------- GET TEMP FUNCTION ---------------------- */
float getTemp() {

    // read temp data
    uint16_t rtd = sensor.readRTD();

    Serial.print("RTD value: "); Serial.println(rtd);

    float ratio = rtd;
    ratio /= 32768;
    Serial.print("Ratio = "); Serial.println(ratio,8);

    Serial.print("Resistance = "); Serial.println(RREF*ratio,8);

    float temp = sensor.temperature(100, RREF);
    Serial.print("Temperature = "); Serial.println(temp);

    // Check and print any faults
    uint8_t fault = sensor.readFault();
    if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold");
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold");
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias");
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
        }
        if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage");
        }
        sensor.clearFault();
    }
    Serial.println();
    delay(1000);

    return temp;

}

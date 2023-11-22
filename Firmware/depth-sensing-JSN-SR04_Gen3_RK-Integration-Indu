#include "JSN-SR04_Gen3_RK.h"

const int TRIG_PIN = A2;
const int ECHO_PIN = A1;
const int UNUSED_PIN0 = A0;
const int UNUSED_PIN3 = A3;

// ------------------ Global Variables --------------------------------
float temperatureC;
float depth;

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
int sensorPin = A4; //the analog pin the TMP36's Vout (sense) pin is connected to
#include <Adafruit_MAX31865.h>
const int TEMP_CHIP_SELECT = D6;
Adafruit_MAX31865 sensor = Adafruit_MAX31865(TEMP_CHIP_SELECT);
#define RREF 430.0 //Rref resistor value = 430.0


//-------------------------- LED Setup -------------------------------------------
const pin_t MY_LED = D7; // blink to let us know you're alive
bool led_state = HIGH; // starting state


// Global objects; TODO: save power stats
FuelGauge batteryMonitor;

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// Declare one of these as a global variable to manage the JSN-SR04 sensor
JSN_SR04_Gen3 distanceSensor;


//===============================================================================
// INITIALIZATION
//===============================================================================
void setup() {

    Serial.begin(115200);

    // Initialize the sensor configuration from setup()
    distanceSensor
        .withTrigPin(TRIG_PIN)
        .withEchoPin(ECHO_PIN)
        .withUnusedPins(UNUSED_PIN0, UNUSED_PIN3)
        .withCallback(distanceCallback)
        .withSamplePeriodic(500ms)
        .setup();

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
    // You must call this frequently from loop(), preferable on every execution
    distanceSensor.loop();
   
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

        temperatureC = getTemp();
        JSN_SR04_Gen3::DistanceResult result = distanceSesnor.getDistance();
        depth = distanceCallback(result);

        // If GPS gets a fix, print out and save good data
        if (GPS.fix) {  
            // Blink to let us know you're alive
            led_state = !led_state;
            digitalWrite(MY_LED, led_state); // turn the LED on (HIGH is the voltage level)  

            serialPrintGPSLoc();
            

        }

        printToFile(); 

    }



/* ---------------------- DISTNACE CALLBACK FUNCTION (printing---------------------- */
float distanceCallback(JSN_SR04_Gen3::DistanceResult result) {
    switch(result.status) {
        case JSN_SR04_Gen3::DistanceResult::Status::SUCCESS:
            Serial.printf("cm=%f inch=%f", result.cm(), result.inch());
            depth = result.cm();
            break;

        case JSN_SR04_Gen3::DistanceResult::Status::RANGE_ERROR:
            Serial.print("distance range error");
            break;

        default:
            Serial.printf("distance error %f", result.status);
            break;
    }

    return depth;
}

/* ---------------------- GET TEMP FUNCTION ---------------------- */
float getTemp() {

    //getting the voltage reading from the temperature sensor
    int reading = analogRead(sensorPin);  

    Serial.println(reading);
    
    // converting that reading to voltage, for 3.3v arduino use 3.3
    float voltage = reading * 3.3;
    voltage /= 4096.0; 
    
    // print out the voltage
    Serial.print(voltage); Serial.println(" volts");
    
    // now print out the temperature
    float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                                  //to degrees ((voltage - 500mV) times 100)
    Serial.print(temperatureC); Serial.println(" degrees C");

    return temperatureC;

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

    // Temperature
    dataFile.print(temperatureC);
    dataFile.print(",");

    // Depth/distance
    dataFile.print(depth);
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

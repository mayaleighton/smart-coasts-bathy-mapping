/*
 * JSN_SR04T Ver 3.0 Ultrasonic Test - Serial Interface Mode
 * from https://protosupplies.com/product/jsn-sr04t-v3-0-waterproof-ultrasonic-range-finder/
 * adapted by PJB 7-July-2023. PJB's is intended for Particle HW without software serial.
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

// To use Particle devices without cloud connectivity
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

byte StartByte = 0;           // 0xFF signifies a new packet of info coming in
byte MSByte = 0;              // Most Significant Byte of distance measurement
byte LSByte = 0;              // Least Significant Byte of distance measurement
byte CheckSum = 0;            // CheckSum = 0xFF + MSByte + LSByte
unsigned int mmDist = 0;      // Returned distance in millimeters

//===============================================================================
//  Initialization
//=============================================================================== 
void setup() {
    Serial.begin(9600);       
    Serial1.begin(9600);    // Initialize the software serial port
}

//===============================================================================
//  Main Loop
//=============================================================================== 
void loop() {
    Serial1.flush();        // Clear the serial port buffer   
    Serial1.write(0x55);    // Request distance measurement
    delay(100);               // Give sensor time to make the measurement (>60mSec)
 
    if(Serial1.available() >= 4){    // Looking for 4 bytes to be returned
      StartByte = Serial1.read();    // Read first byte 
      if (StartByte == 0xFF){          // 1st byte is 0xFF for new measurement
        MSByte = Serial1.read();     // Read in the MSB (Most Significant Byte)
        LSByte  = Serial1.read();    // Read in the LSB (Least Significant Byte)
        CheckSum = Serial1.read();   // Read the checksum byte
        mmDist = MSByte * 256 + LSByte;// Calculate the distance from the two bytes
        Serial.print("Distance: "); 
        Serial.print(mmDist);          // Print in millimeters
        Serial.print("mm  /  "); 
        Serial.print(mmDist/25.4, 2);   // Print in inches
        Serial.println("in");
        }
      else{                           
        Serial1.flush();              // Flush the buffer until valid start byte
      }
    }
    else{
      Serial.println("no serial yet"); // let us know if 4 bytes are available
    }
    delay(900); // slow to ~ 1 Hz readout (including 100 ms delay above)
    
}
// Project AP 649 
// (c) Aidan, Carlo, Jason
//
// Create a simple Realtime bluetoth Serial connection for sensor reading data to client via json data
//
// Feather ESP32 Firmware v1.0 - Data Visualization
// 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#include "BluetoothSerial.h"


// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();



#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5


// Max Diff between last reading and next (Prevent Excessive Data Traffic)
#define MAX_DIFF 0.2


// Warning if no BT configured or enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Create bluetooth serial object for BT UART Connection
BluetoothSerial SerialBT;



void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

// Delta from last reading
float delta[4];


///////////////////////
// Example SPI Setup //
///////////////////////
// Define the pins used for our SPI chip selects. We're
// using hardware SPI, so other signal pins are set in stone.
#define LSM9DS1_M_CS  10 // Can be any digital pin
#define LSM9DS1_AG_CS 9  // Can be any other digital pin

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.




// Initialize hardware for acccel reading and BT connection
void setup() {
  Serial.begin(115200);

  Serial.println("LSM9DS1 data read demo");

  delay(250);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  
  // Initialize the random seed value with analog ref data.
  randomSeed(analogRead(0));

  // Clear previous readings
  delta[0] = 0;
  delta[1] = 0;
  delta[2] = 0;

  // Assign a BT Name to the device for clients to identify the device
  SerialBT.begin("AP649"); //Bluetooth device name
  Serial.println("The device is ready, bluetooth activated!");
  
}

// Boolean state for BTClient connected.
byte btCient=false;

// Serial debug, lower output sample rate (counter)
byte xa = 0;

byte x = 0;



///////////////////////////////////////
// Main Firmware loop
///////////////////////////////////////
void loop() {

    lsm.read();  /* ask it to read in the data */ 
 
    
    // Dirty Data Flag, true produces BT Serial data output
    byte dirty = false;
    String outPut = "";
/* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 
  
      outPut += "{\"X\":";
      outPut += a.acceleration.x*5;
      outPut += ",\"Y\":";
      outPut += a.acceleration.y*5;
      outPut += ",\"Z\":";
      outPut += a.acceleration.z*5;
      outPut += ",\"GyroX\":";
      outPut += g.gyro.x;
      outPut += ",\"GyroY\":";
      outPut += g.gyro.y;
      outPut += ",\"GyroZ\":";
      outPut += g.gyro.z;
      outPut += "}\r\n";
      

  //delay(200);


    if(x!=a.acceleration.x) 
    {
      x = a.acceleration.x;
      dirty=true;
    }
    
    if(dirty)
    {


      xa++;
      
      if(xa>2) {
      // Local Debug Serial Port Monitor
      Serial.print(outPut);
        xa  = 0;
      }
      
      // Detect a connected BT Client, Write json data to BT Client
      if (SerialBT.hasClient() == true) {
        
        // Write json to BT Client
        SerialBT.print(outPut);
     
      } else {
        //SerialBT.hasClient()
        //btCient = false;
      }
      
    } // end of dirty flag
     

  // Optional Delays for Dirty records vs Clean (No Movement) data.
  if(dirty) 
    delay(10); // Fast Data Mode
  else
    delay(200); // Slow Data Mode
}

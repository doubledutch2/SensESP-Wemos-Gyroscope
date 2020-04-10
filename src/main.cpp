#include <Arduino.h>

//#define SERIAL_DEBUG_DISABLED

#define USE_LIB_WEBSOCKET true

#include "sensesp_app.h"
//  #include "sensors/onewire_temperature.h"
#include "signalk/signalk_output.h"
#include "wiring_helpers.h"
#include "sensors/i2c_tools.h"
#include "transforms/linear.h"
#include "SensMPU9250.h"

#define  I2C_ADDRESS 0x68
// MPU9250
/*
#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS 0x68   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

void   setupMPU9250();

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
// End MPU9250
*/
ReactESP app([] () {
  #ifndef SERIAL_DEBUG_DISABLED
  Serial.begin(115200);


  // A small arbitrary delay is required to let the
  // serial port catch up

  delay(100);
  Debug.setSerialEnabled(true);
  #endif

  boolean disableSystemSensors = true;
  sensesp_app = new SensESPApp(disableSystemSensors);

  //  setupMPU9250();

  /* Find all the sensors and their unique addresses. Then, each new instance
     of OneWireTemperature will use one of those addresses. You can't specify
     which address will initially be assigned to a particular sensor, so if you
     have more than one sensor, you may have to swap the addresses around on
     the configuration page for the device. (You get to the configuration page
     by entering the IP address of the device into a browser.)
  */
  // DallasTemperatureSensors* dts = new DallasTemperatureSensors(D7);

  uint read_delay = 5000;
  scan_i2c();
  
  // MPU9250 myIMU(0x68, Wire, 40000);
  auto* mMPU9250 = new mpu9250(I2C_ADDRESS,read_delay,"/ahrs/mpu9250");
  
  /*
  auto* pCoolantTemp = new OneWireTemperature(dts, read_delay, "/coolantTemperature/oneWire");

    pCoolantTemp->connectTo(new Linear(1.0, 0.0, "/coolantTemperature/linear"))
                ->connectTo(new SKOutputNumber("propulsion.mainEngine.coolantTemperature", "/coolantTemperature/skPath"));

  auto* pExhaustTemp = new OneWireTemperature(dts, read_delay, "/exhaustTemperature/oneWire");
    
    pExhaustTemp->connectTo(new Linear(1.0, 0.0, "/exhaustTemperature/linear"))
                ->connectTo(new SKOutputNumber("propulsion.mainEngine.exhaustTemperature", "/exhaustTemperature/skPath"));
  
  auto* p24VTemp = new OneWireTemperature(dts, read_delay, "/24vAltTemperature/oneWire");
      
      p24VTemp->connectTo(new Linear(1.0, 0.0, "/24vAltTemperature/linear"))
              ->connectTo(new SKOutputNumber("electrical.alternators.24V.temperature", "/24vAltTemperature/skPath"));

  auto* p12VTemp = new OneWireTemperature(dts, read_delay, "/12vAltTemperature/oneWire");
      
      p12VTemp->connectTo(new Linear(1.0, 0.0, "/12vAltTemperature/linear"))
              ->connectTo(new SKOutputNumber("electrical.alternators.12V.temperature", "/12vAltTemperature/skPath"));      
  */

  sensesp_app->enable();
});


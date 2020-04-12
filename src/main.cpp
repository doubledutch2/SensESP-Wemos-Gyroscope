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
  auto* mMPU9250 = new mpu9250(I2C_ADDRESS,read_delay,"/ahrs/mpu9250"); // Create an instance of the class and start reading data. We only do that once
  
  auto* pMPU9250yaw = new mpu9250value(mMPU9250, yaw, read_delay, "/motion/yaw");    
        pMPU9250yaw->connectTo(new SKOutputNumber("vessel.motion.yaw","/yaw/skPath"));
  
  auto* pMPU9250pitch = new mpu9250value(mMPU9250, pitch, read_delay, "/motion/pitch");    
        pMPU9250pitch->connectTo(new SKOutputNumber("vessel.motion.pitch","/pitch/skPath"));

  auto* pMPU9250roll = new mpu9250value(mMPU9250, roll, read_delay, "/motion/roll");    
        pMPU9250roll->connectTo(new SKOutputNumber("vessel.motion.roll","/yaw/skPath"));

  auto* pMPU9250xAcc = new mpu9250value(mMPU9250, xAcc, read_delay, "/motion/xAcc");    
        pMPU9250xAcc->connectTo(new SKOutputNumber("vessel.motion.xAcc","/xAcc/skPath"));

  auto* pMPU9250yAcc = new mpu9250value(mMPU9250, yAcc, read_delay, "/motion/yAcc");    
        pMPU9250yAcc->connectTo(new SKOutputNumber("vessel.motion.yAcc","/yAcc/skPath"));

  auto* pMPU9250zAcc = new mpu9250value(mMPU9250, zAcc, read_delay, "/motion/zAcc");    
        pMPU9250zAcc->connectTo(new SKOutputNumber("vessel.motion.zAcc","/zAcc/skPath"));

  auto* pMPU9250xGyro = new mpu9250value(mMPU9250, xGyro, read_delay, "/motion/xGyro");    
        pMPU9250xGyro->connectTo(new SKOutputNumber("vessel.motion.xGyro","/xGyro/skPath"));

  auto* pMPU9250yGyro = new mpu9250value(mMPU9250, yGyro, read_delay, "/motion/yGyro");    
        pMPU9250yGyro->connectTo(new SKOutputNumber("vessel.motion.yGyro","/yGyro/skPath"));

  auto* pMPU9250zGyro = new mpu9250value(mMPU9250, zGyro, read_delay, "/motion/zGyro");    
        pMPU9250zGyro->connectTo(new SKOutputNumber("vessel.motion.zGyro","/zGyro/skPath"));

  auto* pMPU9250xMag = new mpu9250value(mMPU9250, xMag, read_delay, "/motion/xMag");    
        pMPU9250xMag->connectTo(new SKOutputNumber("vessel.motion.xMag","/xMag/skPath"));
  
  auto* pMPU9250yMag = new mpu9250value(mMPU9250, yMag, read_delay, "/motion/yMag");    
        pMPU9250yMag->connectTo(new SKOutputNumber("vessel.motion.yMag","/yMag/skPath"));

  auto* pMPU9250zMag = new mpu9250value(mMPU9250, zMag, read_delay, "/motion/zMag");    
        pMPU9250zMag->connectTo(new SKOutputNumber("vessel.motion.zMag","/zMag/skPath"));

  auto* pMPU9250temperature = new mpu9250value(mMPU9250, temperature, read_delay, "/motion/temperature");    
        pMPU9250temperature->connectTo(new SKOutputNumber("vessel.motion.temperature","/temperature/skPath"));
  

  sensesp_app->enable();
});


#include "SensMPU9250.h"
// #include <RemoteDebug.h> 
//  #include "sensesp.h"


#define SerialDebug false  // Set to true to get Serial output for debugging

// mpu9250 represents 9 Axis Gyro main code from: https://learn.sparkfun.com/tutorials/mpu-9250-hookup-guide/all
// The mpu9250 Class is set up to scan the sensor every few milli seconds both AHRS Raw data

mpu9250::mpu9250(uint8_t addr, uint read_delay, String config_path) :
       Sensor(config_path), 
       addr{addr},
       read_delay{read_delay}
       {
    MPU9250 myIMU(addr);

    className = "MPU9250";
    load_configuration();
    if (!check_status()) {
      debugI("Error initializing MPU9250");
    }
    else {
      app.onRepeat(5, [this](){   //  Every 5ms
        read_values(true);        //  Get AHRS value (Altitude, Heading, Reference System)
        read_values(false);       //  Get Raw data
      });        
    }
}

boolean mpu9250::check_status() {
  Wire.begin();

  while(!Serial){};

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(addr, WHO_AM_I_MPU9250);
  debugI("MPU9250 I AM 0x%x",HEX);
  
  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    debugI("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    debugI("x-axis self test: acceleration trim within : %.2f%% of factory value",myIMU.selfTest[0]); 
    debugI("y-axis self test: acceleration trim within : %.2f%% of factory value",myIMU.selfTest[1]); 
    debugI("z-axis self test: acceleration trim within : %.2f%% of factory value",myIMU.selfTest[2]); 
    debugI("x-axis self test: gyration trim within : %.2f%% of factory value",myIMU.selfTest[3]); 
    debugI("y-axis self test: gyration trim within : %.2f%% of factory value",myIMU.selfTest[4]); 
    debugI("z-axis self test: gyration trim within : %.2f%% of factory value",myIMU.selfTest[5]); 

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    debugI("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    debugI("AK8963 I AM 0x%x (should be 0x48)",HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      debugI("Communication failed, abort!");
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    debugI("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  debugI("Calibration values: ");
      debugI("X-Axis factory sensitivity adjustment value %.2f",myIMU.factoryMagCalibration[0]);
      debugI("Y-Axis factory sensitivity adjustment value %.2f",myIMU.factoryMagCalibration[1]);
      debugI("Z-Axis factory sensitivity adjustment value %.2f",myIMU.factoryMagCalibration[2]);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.

    debugI("AK8963 mag biases (mG) %.2f %.2f %.2f",myIMU.magBias[0],myIMU.magBias[1],myIMU.magBias[2]);
    debugI("AK8963 mag scale  (mG) %.2f %.2f %.2f",myIMU.magScale[0],myIMU.magScale[1],myIMU.magScale[2]);
 
    if(SerialDebug)
    {
      debugI("Magnetometer:");
      debugI("X-Axis sensitivity adjustment value %.2f",myIMU.factoryMagCalibration[0]);
      debugI("Y-Axis sensitivity adjustment value %.2f",myIMU.factoryMagCalibration[1]);
      debugI("Z-Axis sensitivity adjustment value %.2f",myIMU.factoryMagCalibration[2]);
    }

  } // if (c == 0x71)
  else
  {
    debugI("Could not connect to MPU9250: 0x%x - Communication Failed", HEX);
    // Communication failed, stop here
    //  Serial.flush();
    return false;
  }
  return true;
}

void mpu9250::read_values(boolean AHRS)
{

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(addr, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  else {
    // debugI("Interupt not set");
  }
  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
 
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > read_delay)
    {
      SensorRead = true;
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        debugI("X-acceleration: %.2f mg",1000 * myIMU.ax);
        debugI("Y-acceleration: %.2f mg",1000 * myIMU.ay);
        debugI("Z-acceleration: %.2f mg",1000 * myIMU.az);

        // Print gyro values in degree/sec
        debugI("X-gyro rate: %.3f degrees/sec",myIMU.gx);
        debugI("Y-gyro rate: %.3f degrees/sec",myIMU.gy);
        debugI("Z-gyro rate: %.3f degrees/sec",myIMU.gz);

        // Print mag values in milligauss
        debugI("X-mag field: %.2f mG",myIMU.mx);
        debugI("Y-mag field: %.2f mG",myIMU.my);
        debugI("Z-mag field: %.2f mG",myIMU.mz);

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        debugI("Temperature is %.1f degrees C",myIMU.temperature);
      }


      myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    SensorRead = true;
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > read_delay)
    {

      if(SerialDebug)
      {
        /*
        debugI("ax = ");  debugI((int)1000 * myIMU.ax);
        debugI(" ay = "); debugI((int)1000 * myIMU.ay);
        debugI(" az = "); debugI((int)1000 * myIMU.az);
        debugI(" mg");

        debugI("gx = ");  debugI(myIMU.gx, 2);
        debugI(" gy = "); debugI(myIMU.gy, 2);
        debugI(" gz = "); debugI(myIMU.gz, 2);
        debugI(" deg/s");

        debugI("mx = ");  debugI((int)myIMU.mx);
        debugI(" my = "); debugI((int)myIMU.my);
        debugI(" mz = "); debugI((int)myIMU.mz);
        debugI(" mG");

        debugI("q0 = ");  debugI(*getQ());
        debugI(" qx = "); debugI(*(getQ() + 1));
        debugI(" qy = "); debugI(*(getQ() + 2));
        debugI(" qz = "); debugI(*(getQ() + 3));
        */
      }

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive 
      // - z-axis is down toward Earth. 
      // - Yaw is the angle between Sensors x-axis and Earth magnetic North (or true North if corrected for local
      //   declination, looking down on the sensor positive yaw is counterclockwise.
      // - Pitch is angle between sensor x-axis and Earth ground plane, toward the
      //   Earth is positive, up toward the sky is negative. 
      // - Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      //   arise from the definition of the homogeneous rotation matrix constructed
      //   from quaternions. 
      // Tait-Bryan angles as well as Euler angles are non-commutative; //
      // that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.

      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      // myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        debugI("Yaw (Angle between X-Axis and Magnetic North)      : %.2f",myIMU.yaw);
        debugI("Pitch (Angle between X-Axis and Earth Ground Plane): %.2f",myIMU.pitch);
        debugI("Roll (Angle beween Y-Axis and Earth Ground Plane)  : %.2f",myIMU.roll);

      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

/*

mpu9250 Class is used to get the Values to Signal K. You call it with one of the
val_type (e.g. yaw) and it then reads the myIMU instance which has all the values in it
and sends that back to SignalK

*/
mpu9250value::mpu9250value( mpu9250* pMPU9250, 
                            MPU9250ValType val_type, 
                            uint read_delay, 
                            String config_path):
    NumericSensor(config_path), pMPU9250{pMPU9250}, val_type{val_type}, read_delay{read_delay} {
    className = "mpu9250value";
    load_configuration();
}

void mpu9250value::enable() {
  
  app.onRepeat(read_delay, [this](){
      switch (val_type) {
        case (yaw):           output = round2Decs(pMPU9250->myIMU.yaw);         break;
        case (pitch):         output = round2Decs(pMPU9250->myIMU.pitch);       break;
        case (roll):          output = round2Decs(pMPU9250->myIMU.roll);        break;
        case (xAcc):          output = round2Decs(pMPU9250->myIMU.ax);          break;
        case (yAcc):          output = round2Decs(pMPU9250->myIMU.ay);          break;    
        case (zAcc):          output = round2Decs(pMPU9250->myIMU.az);          break;
        case (xGyro):         output = round2Decs(pMPU9250->myIMU.gx);          break;
        case (yGyro):         output = round2Decs(pMPU9250->myIMU.gy);          break;
        case (zGyro):         output = round2Decs(pMPU9250->myIMU.gz);          break;
        case (xMag):          output = round2Decs(pMPU9250->myIMU.mx);          break;
        case (yMag):          output = round2Decs(pMPU9250->myIMU.my);          break;
        case (zMag):          output = round2Decs(pMPU9250->myIMU.mz);          break;       
        case (temperature):   output = round2Decs(pMPU9250->myIMU.temperature); break;
        default:              output = 0.0;                                     break;

      }
      notify();    //
  });
}

float mpu9250value::round2Decs(float input) 
{
    float tmpFloat;
    tmpFloat = input * 100;
    tmpFloat = int(tmpFloat);
    tmpFloat = tmpFloat / 100;
    return tmpFloat;
}

JsonObject& mpu9250value::get_configuration(JsonBuffer& buf) {
  JsonObject& root = buf.createObject();
  root["read_delay"] = read_delay;
  root["value"] = output;
  return root;
  };

  static const char SCHEMA[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "read_delay": { "title": "Read delay", "type": "number", "description": "The time, in milliseconds, between each read of the input" },
        "value": { "title": "Last value", "type" : "number", "readOnly": true }
    }
  })###";


  String mpu9250value::get_config_schema() {
  return FPSTR(SCHEMA);
}

bool mpu9250value::set_configuration(const JsonObject& config) {
  String expected[] = {"read_delay"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  read_delay = config["read_delay"];
  return true;
}
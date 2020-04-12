# Wemos-D1-Mini-Gyroscope
MPU-9250 Gyroscope Sensor for SensESP

This code keeps the sensors in your main source director, not the SensESP Sensor directory to keep
my stuff away from the main SensESP repository.

Leon de Beer: 12 April 2020

 by: Based on software by Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 
 
 Breakout --------- Arduino
 
 VCC ---------------------- 3.3V
 
 SDA ----------------------- Check your device - Wemos D2
 
 SCL ----------------------- Check your device - Wemos D1
 
 GND ---------------------- GND

Definitions

Thinking about a sailing yacht:

- (Gravity) Z-Axis If it sinks: You move down the +z axis
- (Yaw) X-Axis: Is the angle from the north pole (well true north)
- (Pitch) ??: Is the angle between the sea and the bow. Riding the waves: Increases the pitch and fallind down a wave reduced it
- (Roll) ??: Is the heel angle - how far does you sail lean over to the port / starboard

Here is a great explenation about how these sensors work: https://github.com/kriswiner/MPU6050/wiki/Affordable-9-DoF-Sensor-Fusion
And Kris Winer's Repo is here: https://github.com/kriswiner/MPU9250

This sensor returns the following data:

Yaw - Degrees
Pitch - Degrees
Roll - Degrees
X-Acceleration - milli g-force
Y-Acceleration - milli g-force
Z-Acceleration - milli g-force
X-Gyro - Degrees/Sec
Y-Gyro - Degrees/Sec
Z-Gyro - Degrees/Sec
X-Mag - milligauss
Y-Mag - milligauss
Z-Mag - milligauss
Temperature - Celsius

These measurements are obtrained every few milliseconds but then, using various algorithms
only exposed to Signal-K every "read_delay" interval

Code gathers samples every 5ms, this happens in the instance of mMPU9250
The values are then send to SignalK every read_delay ms through: mpu9250value

- MPU9250 interupts are not used
- The Sensor is to be kept away from the MPU to avoid interference
- Make sure youre MPU9250 is calibrated. Here are some good tutorials:

https://learn.adafruit.com/adxl345-digital-accelerometer/programming
https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

- And here is an arduino sketch you can use to calibrate (at the bottom of this URL):

https://github.com/bolderflight/MPU9250/issues/33


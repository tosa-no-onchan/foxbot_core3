/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */
// Update by nishi 2021.7.5

#ifndef TURTLEBOT3_SENSOR_H_
#define TURTLEBOT3_SENSOR_H_


//#define Teensy_ICM

#ifdef Teensy_ICM
#include <inttypes.h>
#include <Arduino.h>
//#include <SPI.h>
#include <Teensy-ICM-20948.h>

#else
#include <IMU.h>
//#include <MPU9250_asukiaaa.h>
//#include <SparkFunMPU9250-DMP.h>

#endif

#ifndef TRUE
#define TRUE true
#endif
#ifndef FALSE
#define FALSE false
#endif


#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#if defined NOETIC_SUPPORT
  #include <sensor_msgs/BatteryStateNoetic.h>
#else
  #include <sensor_msgs/BatteryState.h>
#endif

// changed by nishi
//#include "OLLO.h"

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8

//#define DEBUG_SERIAL  SerialBT2
// changed by nishi
#define DEBUG_SERIAL  Serial

typedef struct LED_PIN_ARRAY
{
  int front_left;
  int front_right;
  int back_left;
  int back_right;
}LedPinArray;
 
typedef struct SONAR_PIN
{
  int trig;
  int echo;
}SonarPin;

class Turtlebot3Sensor
{
 public:
  Turtlebot3Sensor();
  ~Turtlebot3Sensor();

  bool init(unsigned long baud=115200);
  //bool init(MPU9250_DMP *imu);

  // IMU
  void initIMU(void);
  sensor_msgs::Imu getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);
  void copyIMU(void);

  float* getOrientation(void);
  sensor_msgs::MagneticField getMag(void);

  // Battery
  float checkVoltage(void);

  // Button
  uint8_t checkPushButton(void);

  // Sound
  void melody(uint16_t* note, uint8_t note_num, uint8_t* durations);
  void makeSound(uint8_t index);  

  // Bumper
  void initBumper(void);
  uint8_t checkPushBumper(void);

  // Cliff sensor
  void initIR(void);
  float getIRsensorData(void);

  // Sonar sensor
  void initSonar(void);
  void updateSonar(uint32_t t);
  float getSonarData(void);

  // Illumination sensor
  float getIlluminationData(void);

  // led pattern
  void initLED(void);
  void setLedPattern(double linear_vel, double angular_vel);

  #ifndef Teensy_ICM
  cIMU imu_;
  #endif


 private:
  sensor_msgs::Imu           imu_msg_;
  #if defined NOETIC_SUPPORT
    sensor_msgs::BatteryStateNoetic  battery_state_msg_;
  #else
    sensor_msgs::BatteryState  battery_state_msg_;
  #endif
  sensor_msgs::MagneticField mag_msg_;

  #ifdef Teensy_ICM

  TeensyICM20948Settings icmSettings =
  {
    //.cs_pin = 10,                   // SPI chip select pin
    .cs_pin = 5,                   // SPI chip select pin
    .spi_speed = 7000000,           // SPI clock speed in Hz, max speed is 7MHz
    .mode = 1,                      // 0 = low power mode, 1 = high performance mode
    .enable_gyroscope = false,      // Enables gyroscope output
    .enable_accelerometer = false,  // Enables accelerometer output
    .enable_magnetometer = false,   // Enables magnetometer output
    .enable_quaternion = true,      // Enables quaternion output
    .gyroscope_frequency = 1,       // Max frequency = 225, min frequency = 1
    .accelerometer_frequency = 1,   // Max frequency = 225, min frequency = 1
    .magnetometer_frequency = 1,    // Max frequency = 70, min frequency = 1
    .quaternion_frequency = 225     // Max frequency = 225, min frequency = 50
  };

  TeensyICM20948 imu_;
  float qw, qx, qy, qz;

  #else
  //cIMU imu_;
  // changed by nishi
  //OLLO ollo_;
  #endif

  float quat[4];  // add by nishi 2021.12.25

  LedPinArray led_pin_array_;
  SonarPin sonar_pin_;

  float sonar_data_;

};

#endif // TURTLEBOT3_SENSOR_H_

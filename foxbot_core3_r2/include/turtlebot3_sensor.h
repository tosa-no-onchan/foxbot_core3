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


#include <IMU.h>
//#include <MPU9250_asukiaaa.h>
//#include <SparkFunMPU9250-DMP.h>


#ifndef TRUE
#define TRUE true
#endif
#ifndef FALSE
#define FALSE false
#endif

#ifdef USE_ROS
  #include <sensor_msgs/Imu.h>
  #include <sensor_msgs/MagneticField.h>
#else
  #include <sensor_msgs/msg/imu.h>
  #include <sensor_msgs/msg/magnetic_field.h>
#endif

#if defined NOETIC_SUPPORT
  #include <sensor_msgs/BatteryStateNoetic.h>
#else
  //#include <sensor_msgs/BatteryState.h>
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

  //bool init(unsigned long baud=115200);
  uint8_t init(unsigned long baud=115200);

  // IMU
  uint8_t initIMU(void);
  //sensor_msgs::Imu getIMU(void);
  sensor_msgs__msg__Imu getIMU(void);
  //void updateIMU(void);
  // changed by nishi 2025.3.6
  bool updateIMU(void);
  void calibrationGyro(void);
  void copyIMU(void);

  float* getOrientation(void);
  //sensor_msgs::MagneticField getMag(void);
  sensor_msgs__msg__MagneticField getMag(void);

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

  cIMU cimu_;


 private:
  //sensor_msgs::Imu           imu_msg_;
  sensor_msgs__msg__Imu     imu_msg_;
  #if defined NOETIC_SUPPORT
    sensor_msgs::BatteryStateNoetic  battery_state_msg_;
  #else
    //sensor_msgs::BatteryState  battery_state_msg_;
  #endif
  
  //sensor_msgs::MagneticField mag_msg_;
  sensor_msgs__msg__MagneticField mag_msg_;

  //cIMU imu_;
  // changed by nishi
  //OLLO ollo_;

  float quat[4];  // add by nishi 2021.12.25

  LedPinArray led_pin_array_;
  SonarPin sonar_pin_;

  float sonar_data_;

};

#endif // TURTLEBOT3_SENSOR_H_

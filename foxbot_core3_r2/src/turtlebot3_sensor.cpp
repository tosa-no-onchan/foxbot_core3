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

#include "turtlebot3_sensor.h"

//#define DEBUG_N

Turtlebot3Sensor::Turtlebot3Sensor()
{
}

Turtlebot3Sensor::~Turtlebot3Sensor()
{
  #ifdef DEBUG_N
  DEBUG_SERIAL.end();
  #endif
}

//bool Turtlebot3Sensor::init(unsigned long baud)
uint8_t Turtlebot3Sensor::init(unsigned long baud)
{
  #ifdef DEBUG_N
  DEBUG_SERIAL.begin(baud);
  //DEBUG_SERIAL.begin(115200);
  #endif

  //initBumper();
  //initIR();
  //initSonar();
  //initLED();

  uint8_t get_error_code = 0x00;

  #if defined NOETIC_SUPPORT
    battery_state_msg_.temperature     = NAN;
  #endif

  //battery_state_msg_.current         = NAN;
  //battery_state_msg_.charge          = NAN;
  //battery_state_msg_.capacity        = NAN;
  //battery_state_msg_.design_capacity = NAN;
  //battery_state_msg_.percentage      = NAN;


  get_error_code = cimu_.begin(206);

  // add by nishi 2025.3.20
  quat_[0] = cimu_.quat[0];
  quat_[1] = cimu_.quat[1];
  quat_[2] = cimu_.quat[2];
  quat_[3] = cimu_.quat[3];

  // add by nishi 2025.3.17
  gyroADC_[0] = cimu_.SEN.gyroADC[0];
  gyroADC_[1] = cimu_.SEN.gyroADC[1];
  gyroADC_[2] = cimu_.SEN.gyroADC[2];

  // add by nishi 2025.3.17
  accADC_[0] = cimu_.SEN.accADC[0];
  accADC_[1] = cimu_.SEN.accADC[1];
  accADC_[2] = cimu_.SEN.accADC[2];

  // add by nishi 2025.3.17
  magADC_[0] = cimu_.SEN.magADC[0];
  magADC_[1] = cimu_.SEN.magADC[1];
  magADC_[2] = cimu_.SEN.magADC[2];
  

  #ifdef DEBUG_N
  if (get_error_code != 0x00)
    DEBUG_SERIAL.println("Failed to init Sensor");
  else
    DEBUG_SERIAL.println("Success to init Sensor");
  #endif

  return get_error_code;
}

uint8_t Turtlebot3Sensor::initIMU(void){
  return cimu_.begin(206);
}

#define LED_BUILTIN 17
//#define LED_BUILTIN 4

/*
* IMU device へのデータアクセスを行う。
* cIMU::update() -> cIMU::computeIMU()
* 取り込まれたデータは、cIMU クラス の 各public 変数に保存される。
*/
bool Turtlebot3Sensor::updateIMU(void){
    //cimu_.update();
    // changed by nishi 2025.3.7
    return cimu_.update();
}

/*
* cIMU インスタンス へ取り込まれた IMU データを、一括して、自クラスへコピーする。
*/
void Turtlebot3Sensor::copyIMU(void){
  quat_[0]=cimu_.quat[0];
  quat_[1]=cimu_.quat[1];
  quat_[2]=cimu_.quat[2];
  quat_[3]=cimu_.quat[3];

  // add by nishi 2025.3.17
  gyroADC_[0] = cimu_.SEN.gyroADC[0];
  gyroADC_[1] = cimu_.SEN.gyroADC[1];
  gyroADC_[2] = cimu_.SEN.gyroADC[2];

  // add by nishi 2025.3.17
  accADC_[0] = cimu_.SEN.accADC[0];
  accADC_[1] = cimu_.SEN.accADC[1];
  accADC_[2] = cimu_.SEN.accADC[2];

  // add by nishi 2025.3.17
  magADC_[0] = cimu_.SEN.magADC[0];
  magADC_[1] = cimu_.SEN.magADC[1];
  magADC_[2] = cimu_.SEN.magADC[2];

}

/*
* copyIMU() で、一括コピーした自クラス上のデータをコール元へ渡す。
*/
//sensor_msgs::Imu Turtlebot3Sensor::getIMU(void)
void Turtlebot3Sensor::getIMU(sensor_msgs__msg__Imu &imu_msg)
{
  //imu_msg_.angular_velocity.x = cimu_.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_msg.angular_velocity.x = gyroADC_[0] * GYRO_FACTOR;
  //imu_msg_.angular_velocity.y = cimu_.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_msg.angular_velocity.y = gyroADC_[1] * GYRO_FACTOR;
  //imu_msg_.angular_velocity.z = cimu_.SEN.gyroADC[2] * GYRO_FACTOR;
  imu_msg.angular_velocity.z = gyroADC_[2] * GYRO_FACTOR;

  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  //imu_msg_.linear_acceleration.x = cimu_.SEN.accADC[0] * ACCEL_FACTOR;
  //imu_msg_.linear_acceleration.y = cimu_.SEN.accADC[1] * ACCEL_FACTOR;
  //imu_msg_.linear_acceleration.z = cimu_.SEN.accADC[2] * ACCEL_FACTOR;

  // MPU6500
  //imu_msg_.linear_acceleration.x = cimu_.SEN.accADC[0] * ACCEL_FACTOR - 2.65;
  //imu_msg_.linear_acceleration.y = cimu_.SEN.accADC[1] * ACCEL_FACTOR + 0.8;
  //imu_msg_.linear_acceleration.z = cimu_.SEN.accADC[2] * ACCEL_FACTOR;

  // ICM-20948
  //imu_msg_.linear_acceleration.x = cimu_.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.x = accADC_[0] * ACCEL_FACTOR;
  //imu_msg_.linear_acceleration.y = cimu_.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.y = accADC_[1] * ACCEL_FACTOR;
  //imu_msg_.linear_acceleration.z = cimu_.SEN.accADC[2] * ACCEL_FACTOR;
  imu_msg.linear_acceleration.z = accADC_[2] * ACCEL_FACTOR;

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  //imu_msg_.orientation.w = cimu_.quat[0];
  //imu_msg_.orientation.x = cimu_.quat[1];
  //imu_msg_.orientation.y = cimu_.quat[2];
  //imu_msg_.orientation.z = cimu_.quat[3];

  // MPU6500 & ICM-20948
  imu_msg.orientation.w = quat_[0];
  imu_msg.orientation.x = quat_[1];
  imu_msg.orientation.y = quat_[2];
  imu_msg.orientation.z = quat_[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  //return imu_msg_;
}

void Turtlebot3Sensor::calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  const uint8_t led_ros_connect = 3;

  cimu_.SEN.gyro_cali_start();
  
  t_time   = millis();
  pre_time = millis();

  while(!cimu_.SEN.gyro_cali_get_done())
  {
    cimu_.update();

    if (millis()-pre_time > 5000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      //setLedToggle(led_ros_connect);
    }
  }
}

//float* Turtlebot3Sensor::getOrientation(void)
void Turtlebot3Sensor::getOrientation(double *orientation)
{
  //static float orientation[4];

  //orientation[0] = cimu_.quat[0];
  orientation[0] = quat_[0];
  //orientation[1] = cimu_.quat[1];
  orientation[1] = quat_[1];
  //orientation[2] = cimu_.quat[2];
  orientation[2] = quat_[2];
  //orientation[3] = cimu_.quat[3];
  orientation[3] = quat_[3];

  //return orientation;
}

//sensor_msgs::MagneticField Turtlebot3Sensor::getMag(void)
sensor_msgs__msg__MagneticField Turtlebot3Sensor::getMag(void)
{
  //mag_msg_.magnetic_field.x = cimu_.SEN.magADC[0] * MAG_FACTOR;
  mag_msg_.magnetic_field.x = magADC_[0] * MAG_FACTOR;
  //mag_msg_.magnetic_field.y = cimu_.SEN.magADC[1] * MAG_FACTOR;
  mag_msg_.magnetic_field.y = magADC_[1] * MAG_FACTOR;
  //mag_msg_.magnetic_field.z = cimu_.SEN.magADC[2] * MAG_FACTOR;
  mag_msg_.magnetic_field.z = magADC_[2] * MAG_FACTOR;

  mag_msg_.magnetic_field_covariance[0] = 0.0048;
  mag_msg_.magnetic_field_covariance[1] = 0;
  mag_msg_.magnetic_field_covariance[2] = 0;
  mag_msg_.magnetic_field_covariance[3] = 0;
  mag_msg_.magnetic_field_covariance[4] = 0.0048;
  mag_msg_.magnetic_field_covariance[5] = 0;
  mag_msg_.magnetic_field_covariance[6] = 0;
  mag_msg_.magnetic_field_covariance[7] = 0;
  mag_msg_.magnetic_field_covariance[8] = 0.0048;

  return mag_msg_;
}


float Turtlebot3Sensor::checkVoltage(void)
{
  float vol_value;
  
  //vol_value = getPowerInVoltage();
  vol_value = 0.0;

  return vol_value;
}

uint8_t Turtlebot3Sensor::checkPushButton(void)
{
  //return getPushButton();
  return 0;
}

void Turtlebot3Sensor::melody(uint16_t* note, uint8_t note_num, uint8_t* durations)
{
  //for (int thisNote = 0; thisNote < note_num; thisNote++) 
  //{
    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    //int noteDuration = 1000 / durations[thisNote];
    //tone(BDPIN_BUZZER, note[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    //int pauseBetweenNotes = noteDuration * 1.30;
    //delay(pauseBetweenNotes);
    // stop the tone playing:
    //noTone(BDPIN_BUZZER);
  //}
}

void Turtlebot3Sensor::makeSound(uint8_t index)
{
  const uint16_t NOTE_C4 = 262;
  const uint16_t NOTE_D4 = 294;
  const uint16_t NOTE_E4 = 330;
  const uint16_t NOTE_F4 = 349;
  const uint16_t NOTE_G4 = 392;
  const uint16_t NOTE_A4 = 440;
  const uint16_t NOTE_B4 = 494;
  const uint16_t NOTE_C5 = 523;
//  const uint16_t NOTE_C6 = 1047;

  const uint8_t OFF         = 0;
  const uint8_t ON          = 1;
  const uint8_t LOW_BATTERY = 2;
  const uint8_t ERROR       = 3;
  const uint8_t BUTTON1     = 4;
  const uint8_t BUTTON2     = 5;

  uint16_t note[8]     = {0, 0};
  uint8_t  duration[8] = {0, 0};

  switch (index)
  {
    case ON:
      note[0] = NOTE_C4;   duration[0] = 4;
      note[1] = NOTE_D4;   duration[1] = 4;
      note[2] = NOTE_E4;   duration[2] = 4;
      note[3] = NOTE_F4;   duration[3] = 4;
      note[4] = NOTE_G4;   duration[4] = 4;
      note[5] = NOTE_A4;   duration[5] = 4;
      note[6] = NOTE_B4;   duration[6] = 4;
      note[7] = NOTE_C5;   duration[7] = 4;   
     break;

    case OFF:
      note[0] = NOTE_C5;   duration[0] = 4;
      note[1] = NOTE_B4;   duration[1] = 4;
      note[2] = NOTE_A4;   duration[2] = 4;
      note[3] = NOTE_G4;   duration[3] = 4;
      note[4] = NOTE_F4;   duration[4] = 4;
      note[5] = NOTE_E4;   duration[5] = 4;
      note[6] = NOTE_D4;   duration[6] = 4;
      note[7] = NOTE_C4;   duration[7] = 4;  
     break;

    case LOW_BATTERY:
      note[0] = 1000;      duration[0] = 1;
      note[1] = 1000;      duration[1] = 1;
      note[2] = 1000;      duration[2] = 1;
      note[3] = 1000;      duration[3] = 1;
      note[4] = 0;         duration[4] = 8;
      note[5] = 0;         duration[5] = 8;
      note[6] = 0;         duration[6] = 8;
      note[7] = 0;         duration[7] = 8;
     break;

    case ERROR:
      note[0] = 1000;      duration[0] = 3;
      note[1] = 500;       duration[1] = 3;
      note[2] = 1000;      duration[2] = 3;
      note[3] = 500;       duration[3] = 3;
      note[4] = 1000;      duration[4] = 3;
      note[5] = 500;       duration[5] = 3;
      note[6] = 1000;      duration[6] = 3;
      note[7] = 500;       duration[7] = 3;
     break;

    case BUTTON1:
     break;

    case BUTTON2:
     break;

    default:
      note[0] = NOTE_C4;   duration[0] = 4;
      note[1] = NOTE_D4;   duration[1] = 4;
      note[2] = NOTE_E4;   duration[2] = 4;
      note[3] = NOTE_F4;   duration[3] = 4;
      note[4] = NOTE_G4;   duration[4] = 4;
      note[5] = NOTE_A4;   duration[5] = 4;
      note[6] = NOTE_B4;   duration[6] = 4;
      note[7] = NOTE_C4;   duration[7] = 4; 
     break;
  }

  melody(note, 8, duration);
}

void Turtlebot3Sensor::initBumper(void)
{
  //ollo_.begin(3, TOUCH_SENSOR);
  //ollo_.begin(4, TOUCH_SENSOR);
}

uint8_t Turtlebot3Sensor::checkPushBumper(void)
{
  uint8_t push_state = 0;

  //if      (ollo_.read(3, TOUCH_SENSOR) == HIGH) push_state = 2;
  //else if (ollo_.read(4, TOUCH_SENSOR) == HIGH) push_state = 1;
  //else    push_state = 0;
  
  return push_state;
}

void Turtlebot3Sensor::initIR(void)
{
  //ollo_.begin(2, IR_SENSOR);
}

float Turtlebot3Sensor::getIRsensorData(void)
{
  //float ir_data = ollo_.read(2, IR_SENSOR);
  float ir_data = 0.;
  
  return ir_data;
}

void Turtlebot3Sensor::initSonar(void)
{
  //sonar_pin_.trig = BDPIN_GPIO_1;
  //sonar_pin_.echo = BDPIN_GPIO_2;

  //pinMode(sonar_pin_.trig, OUTPUT);
  //pinMode(sonar_pin_.echo, INPUT);
}

void Turtlebot3Sensor::updateSonar(uint32_t t)
{
  static uint32_t t_time = 0;
  static bool make_pulse = TRUE;
  static bool get_duration = FALSE;

  float distance = 0.0, duration = 0.0;

  if (make_pulse == TRUE)
  {
    //digitalWrite(sonar_pin_.trig, HIGH);

    if (t - t_time >= 10)
    {
      //digitalWrite(sonar_pin_.trig, LOW);

      get_duration = TRUE;
      make_pulse = FALSE;

      t_time = t;
    }
  }

  if (get_duration == TRUE)
  {
    //duration = pulseIn(sonar_pin_.echo, HIGH);
    distance = ((float)(340 * duration) / 10000) / 2;

    make_pulse = TRUE;
    get_duration = FALSE;
  }

  sonar_data_ = distance;
}

float Turtlebot3Sensor::getSonarData(void)
{
  float distance = 0.0;

  distance = sonar_data_;

  return distance;
}

float Turtlebot3Sensor::getIlluminationData(void)
{
  uint16_t light;

  //light = analogRead(A1);

  return light;
}

void Turtlebot3Sensor::initLED(void)
{
  //led_pin_array_.front_left  = BDPIN_GPIO_4;
  //led_pin_array_.front_right = BDPIN_GPIO_6;
  //led_pin_array_.back_left   = BDPIN_GPIO_8;
  //led_pin_array_.back_right  = BDPIN_GPIO_10;
 
  //pinMode(led_pin_array_.front_left, OUTPUT);
  //pinMode(led_pin_array_.front_right, OUTPUT);
  //pinMode(led_pin_array_.back_left, OUTPUT);
  //pinMode(led_pin_array_.back_right, OUTPUT);
}

void Turtlebot3Sensor::setLedPattern(double linear_vel, double angular_vel)
{
  if (linear_vel > 0.0 && angular_vel == 0.0)     // front
  {
    //digitalWrite(led_pin_array_.front_left, HIGH);
    //digitalWrite(led_pin_array_.front_right, HIGH);
    //digitalWrite(led_pin_array_.back_left, LOW);
    //digitalWrite(led_pin_array_.back_right, LOW);
  }
  else if (linear_vel >= 0.0 && angular_vel > 0.0)  // front left
  {
    //digitalWrite(led_pin_array_.front_left, HIGH);
    //digitalWrite(led_pin_array_.front_right, LOW);
    //digitalWrite(led_pin_array_.back_left, LOW);
    //digitalWrite(led_pin_array_.back_right, LOW);
  }
  else if (linear_vel >= 0.0 && angular_vel < 0.0)  // front right
  {
    //digitalWrite(led_pin_array_.front_left, LOW);
    //digitalWrite(led_pin_array_.front_right, HIGH);
    //digitalWrite(led_pin_array_.back_left, LOW);
    //digitalWrite(led_pin_array_.back_right, LOW);
  }
  else if (linear_vel < 0.0 && angular_vel == 0.0) // back
  {
    //digitalWrite(led_pin_array_.front_left, LOW);
    //digitalWrite(led_pin_array_.front_right, LOW);
    //digitalWrite(led_pin_array_.back_left, HIGH);
    //digitalWrite(led_pin_array_.back_right, HIGH);
  }
  else if (linear_vel <= 0.0 && angular_vel > 0.0)  // back right
  {
    //digitalWrite(led_pin_array_.front_left, LOW);
    //digitalWrite(led_pin_array_.front_right, LOW);
    //digitalWrite(led_pin_array_.back_left, LOW);
    //digitalWrite(led_pin_array_.back_right, HIGH);
  }
  else if (linear_vel <= 0.0 && angular_vel < 0.0)  // back left
  {
    //digitalWrite(led_pin_array_.front_left, LOW);
    //digitalWrite(led_pin_array_.front_right, LOW);
    //digitalWrite(led_pin_array_.back_left, HIGH);
    //digitalWrite(led_pin_array_.back_right, LOW);
  }
  else 
  {
    //digitalWrite(led_pin_array_.front_left, LOW);
    //digitalWrite(led_pin_array_.front_right, LOW);
    //digitalWrite(led_pin_array_.back_left, LOW);
    //digitalWrite(led_pin_array_.back_right, LOW);
  }
}






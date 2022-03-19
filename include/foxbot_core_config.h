/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Core configuration file for foxbot robot.
 *  Author: Matthieu Magnon
 *
 *  Serial communication is enabled with the following command:
 *  rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
 */

#ifndef FOXBOT_CORE_CONFIG_H_
#define FOXBOT_CORE_CONFIG_H_

#if defined(BOARD_DUEUSB)
// enable this line for Arduino Due
#define USE_USBCON
#endif

/* Include librairies */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Timer.h>
#include <MedianFilter.h>

// add by nishi
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#if defined(CAMERA_SYNC)
#include <sensor_msgs/CameraInfo.h>
#endif

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <std_msgs/Empty.h>

#include "turtlebot3_sensor.h"

///
//#include <ros.h>
//#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
//#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
//
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Temperature.h>

#include <time.h>

#define FIRMWARE_VER "1.2.6"


/* Global parameters */
//#define FREQUENCY_RATE 						30 			// [ms] default 50ms
#define FREQUENCY_RATE 						20 			// [ms] default 50ms
#define FREQUENCY_RATE_HZ 						50 			// 20[ms] -> 50[Hz]

//#define FREQUENCY_ODOMETRY 				    150 		// [ms] default 250ms
// test by nishi 2021.10.9
//#define FREQUENCY_ODOMETRY 				    200 		// [ms] default 250ms
//#define FREQUENCY_ODOMETRY 				    190 		// [ms] default 250ms
//#define FREQUENCY_ODOMETRY 				    160 		// [ms] default 250ms

//#define FREQUENCY_ODOMETRY_HZ               30   // 30[hz]  Act 25[hz]
//#define FREQUENCY_ODOMETRY_HZ               15   // 15[hz]  Act 25[hz]
#define FREQUENCY_ODOMETRY_HZ               13.6f   // 13.6[hz]

#define FREQUENCY_ODOMETRY 				    33 		// [ms] default 250ms

#define FREQUENCY_ROSPINONCE_HZ				6		// 150[ms] -> 6.6[hz]
#define FREQUENCY_ROSPINONCE 				150 		// [ms]

//#define FREQUENCY_CONTROLLER 				30 			// [ms] default 50ms
#define FREQUENCY_CONTROLLER 				20 			// [ms] default 50ms
#define FREQUENCY_CONTROLLER_HZ				50 			// 20[ms] -> 50[Hz]

#define FREQUENCY_IMU_PUBLISH_HZ            200  // 5[ms] -> 200[hz]
#define FREQUENCY_IMU_PUBLISH               5           // [ms] default 5ms add by nishi 2021.7.5

#define FREQUENCY_IMU_DATA_HZ               100  // 90[hz]
//#define FREQUENCY_IMU_DATA_HZ               200  // 90[hz]


/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 	3
#define RATE_CONV 							0.0073882 	// [inc] -> [rad]
// 853 (234) inc per wheel rotation
#define RATE_AVERAGE_FILTER_SIZE 			4

/* Rate controller parameters */
#define PWM_MAX				 				4095            // 12 bit
//#define RATE_CONTROLLER_KP 					130.0 		    // default 150
// changed by nishi
#define RATE_CONTROLLER_KP 					110.0 		    // default 150
#define RATE_CONTROLLER_KD 					5000000000000.0 //4500000000000
#define RATE_CONTROLLER_KI 					0.00005 	    //0.00001
#define RATE_INTEGRAL_FREEZE				250
#define RATE_CONTROLLER_MIN_PWM 			-500
#define RATE_CONTROLLER_MAX_PWM 			500

/* Mechanical parameters */
//#define WHEEL_RADIUS 						0.0326 		// [m]  Radius
// changed by nishi
//#define WHEEL_RADIUS 						0.0310 		// [m]  Radius
//#define WHEEL_RADIUS 						0.0300 		// [m]  Radius  by nishi 2021.9.23
#define WHEEL_RADIUS 						0.0295 		// [m]  Radius  by nishi 2021.9.27
// distance between the two wheels
//#define BASE_LENGTH 						0.272 		// [m]  0.288
// changed by nishi
//#define BASE_LENGTH 						0.214 		// [m]  0.196
//#define BASE_LENGTH 						0.210 		// [m]  0.196  by nishi 2021.9.23
//#define BASE_LENGTH 						0.204 		// [m]  0.196  by nishi 2021.9.23
#define BASE_LENGTH 						0.200 		// [m]  0.196  by nishi 2021.9.27

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

/* Define frequency loops */
//Timer _frequency_rate(FREQUENCY_RATE);
//Timer _frequency_odometry(FREQUENCY_ODOMETRY);
//Timer _frequency_rospinonce(FREQUENCY_ROSPINONCE);
//Timer _frequency_controller(FREQUENCY_CONTROLLER);
//Timer _frequency_ver_info(FREQUENCY_ROSPINONCE);
//Timer _frequency_imu(FREQUENCY_IMU_PUBLISH);        // add by nishi 2021.7.5

/* Define median filter for direction */
MedianFilter motor_right_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);
MedianFilter motor_left_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);

// add by nishi start
#define WHEEL_NUM                        2
#define LEFT                             0
#define RIGHT                            1


// add by nishi end


#if defined(BOARD_BLACKPILL) or defined(BOARD_F407VG)
/* Define pins */
// motor A (right)
const byte motorRightEncoderPinA = PB3;  // Encode IN  Digital IN (C1)  --> PB3
const byte motorRightEncoderPinB = PB4;   // Encode IN Digital IN (C2) --> PB4
const byte enMotorRight = PB7;    // moter speed Analog(PWM) OUT  --> PB7
const byte in1MotorRight = PB14;   // Forward Switch  Digital OUT  --> PB14
const byte in2MotorRight = PB13;   // Backward Switch Digital OUT  --> PB13

// motor B (left)
const byte motorLeftEncoderPinA = PB9;  // Encode IN Digital IN (C2)  --> PB9
const byte motorLeftEncoderPinB = PB8;  // Encode IN Digital IN (C1)  --> PB8
const byte enMotorLeft = PB6;      // morter speed Analog(PWM) OUT  --> PB6
const byte in1MotorLeft = PA15;    // Forward Switch  Digital OUT  --> PA15
const byte in2MotorLeft = PB15;    // Backward Switch Digital OUT  --> PB15

#elif defined(BOARD_ESP32)
/* Define pins */
// motor A (right)
const byte motorRightEncoderPinA = 36;  // Encode IN  Digital IN (C1)  --> PB3
const byte motorRightEncoderPinB = 39;   // Encode IN Digital IN (C2) --> PB4
const byte enMotorRight = 33;    // moter speed Analog(PWM) OUT  --> PB7
const byte in1MotorRight = 26;   // Forward Switch  Digital OUT  --> PB14
const byte in2MotorRight = 25;   // Backward Switch Digital OUT  --> PB13

// motor B (left)
const byte motorLeftEncoderPinA = 35;  // Encode IN Digital IN (C2)  --> PB9
const byte motorLeftEncoderPinB = 34;  // Encode IN Digital IN (C1)  --> PB8
const byte enMotorLeft = 13;      // morter speed Analog(PWM) OUT  --> PB6
const byte in1MotorLeft = 14;    // Forward Switch  Digital OUT  --> PA15
const byte in2MotorLeft = 27;    // Backward Switch Digital OUT  --> PB15
#else
/* Define pins */
// motor A (right)
const byte motorRightEncoderPinA = 38;
const byte motorRightEncoderPinB = 34;
const byte enMotorRight = 2;
const byte in1MotorRight = 4;   //26 C1 M1
const byte in2MotorRight = 3;   //28 C2 M2

// motor B (left)
const byte motorLeftEncoderPinA = 26;
const byte motorLeftEncoderPinB = 30;
const byte enMotorLeft = 7;
const byte in1MotorLeft = 6;    //30
const byte in2MotorLeft = 5;    //32
#endif

/* Define motors variables */
// right
volatile int motor_right_inc;
int motor_right_direction;
int motor_right_filtered_direction;
float motor_right_filtered_inc_per_second;
float motor_right_rate_est;
float motor_right_rate_ref;
int motor_right_check_dir;
int motor_right_pwm_rate;
unsigned long motor_right_prev_time;
int pwmMotorRight = 0;
// left
volatile int motor_left_inc;
int motor_left_direction;
int motor_left_filtered_direction;
float motor_left_filtered_inc_per_second;
float motor_left_rate_est;
float motor_left_rate_ref;
int motor_left_check_dir;
int motor_left_pwm_rate;
unsigned long motor_left_prev_time;
int pwmMotorLeft = 0;

/* Define controllers variables */
// right
unsigned long controler_motor_right_prev_time;
float controler_motor_right_prev_epsilon = 0.0;
float controler_motor_right_int = 0.0;
// left
unsigned long controler_motor_left_prev_time;
float controler_motor_left_prev_epsilon = 0.0;
float controler_motor_left_int = 0.0;

/* Mixer variable */
float linear_velocity_ref;
float angular_velocity_ref;
float linear_velocity_est;
float angular_velocity_est;

#if defined(CAMERA_SYNC)
/* Camera Info flag */
bool camera_info_f;
double_t camera_sync_time;   // camera_sync timestamp [sec]
float camera_cap_hz;  // camera caption rate [Hz]

bool prev_ok=false;
double_t prev_cinfo_stamp;
uint32_t prev_cinfo_sec;

#elif defined(CAMERA_SYNC_EX)
/* Camera Info flag */
bool camera_info_f;
double_t camera_sync_time;   // camera_sync timestamp [sec]
float camera_cap_hz;  // camera caption rate [Hz]
#endif


float yaw_est;
unsigned long odom_prev_time;

/* ROS Nodehanlde */
ros::NodeHandle  nh;

/* Prototype function */
void rateControler(const float rate_ref, const float rate_est, int & pwm_rate,
                   unsigned long & prev_time, float & previous_epsilon,
                   float & integral_epsilon);
float runningAverage(float prev_avg, const float val, const int n);
void setMotorRateAndDirection(int pwm_ref, const float rate_ref,
                              const byte enMotor, const byte in1Motor, const byte in2Motor);
// add by nishi start
void publishVersionInfoMsg(void);
void initMoter(void);
void updateOdometry(void);
void updateTFPrefix(bool isConnected);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void initOdom(void);
void initJointStates(void);
void updateVariable(bool isConnected);
ros::Time rosNow();     // add by nishi 2021.7.5
void updateGyroCali(bool isConnected); // add by nishi 2021.11.3
void publishImuMsg(void);   // add by nishi 2021.7.5
#ifdef USE_MAG
void publishMagMsg(void);   // add by nishi 2021.11.4
#endif
// add by nishi end

/* Velocity command subscriber */
// callback function prototype
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

// message
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", commandVelocityCallback);

#if defined(CAMERA_SYNC)
/* for stereo camera */
/* CameraInfo subscriber */
// callback function prototype
void cameraSyncCallback(const sensor_msgs::CameraInfo& camera_info_msg);
ros::Subscriber<sensor_msgs::CameraInfo> camera_sync_sub("/rgb/camera_info", cameraSyncCallback);

#elif defined(CAMERA_SYNC_EX)
/*  for OAK-D=Lite */
void cameraSyncCallback(const sensor_msgs::Temperature& temp_msg);
ros::Subscriber<sensor_msgs::Temperature> camera_sync_sub("/camera/sync", cameraSyncCallback);
#endif


/*******************************************************************************
* Publisher
*******************************************************************************/
// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

// IMU of Turtlebot3    add by nishi 2021.7.5
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

/* Odometry publisher */
//nav_msgs::Odometry odom;
//ros::Publisher odom_publisher("odom", &odom);

// Odometry of Turtlebot3
nav_msgs::Odometry odom;
//ros::Publisher odom_pub("odom", &odom);
// test by nishi 2021.10.8
ros::Publisher odom_pub("odom_fox", &odom);

// add by nishi start
// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

#if defined(USE_MAG)
// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);
#endif

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

struct MOTOR_TIC{
    int32_t left;
    int32_t right;
};
MOTOR_TIC motor_tic;


int32_t dzi[3];
int32_t dzi_sum;

// add by nishi end


/* DEBUG */
//#include <geometry_msgs/Point.h>
geometry_msgs::Point debug_left;
ros::Publisher debug_publisher1("debug_left", &debug_left);

//#include <geometry_msgs/Point.h>
geometry_msgs::Point debug_right;
ros::Publisher debug_publisher2("debug_right", &debug_right);

template <typename type>
type sign(type value) {
    return type((value>0)-(value<0));
}

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10]={0,0,0,0,0,0,0,0,0,0};

uint32_t frequency_odometry_hz;     // FREQUENCY_ODOMETRY_HZ
uint32_t frequency_odometry_hz_ave; // FREQUENCY_ODOMETRY_HZ average

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;   // add by nishi 2021.7.5

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
bool setup_end        = false;
uint8_t battery_state = 0;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

#endif // FOXBOT_CORE_CONFIG_H_

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
#include <micro_ros_arduino.h>
//#include <IMU.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// add by nishi 2023.2.24
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

// add by nishi
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
//#include <std_msgs/msg/int32.h>

//#include <micro_ros_utilities/type_utilities.h>
//#include <micro_ros_utilities/string_utilities.h>
#include <geometry_msgs/msg/vector3.h>
//#include <geometry_msgs/msg/quaternion.h>

//---------------------------------------------------
// change these params for your robot contorol 1
//
// 1) enable whehter 'pc_beat' syncronize or not
//#define USE_PC_BEAT
//#define USE_FOX_BEAT

// 2) use odom topic name which '/odom' or '/odom_fox'  add by nishi 2023.4.4
#define USE_ODOM_FOX

// 3) Debug
#define USE_DEBUG

//----------------------------------------------------

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// add 2023.2.24
rcl_init_options_t init_options;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_CONNECT_WATCH,  // add by nishi 2023.2.27
  AGENT_DISCONNECTED,
  AGENT_INIT       // add by nishi 2023.2.25
};
states state;
states trace_prv;
int agent_connect_check_cnt;
int init_cnt=0;

//#define LED_PIN 13
#define LED_PIN LED_BUILTIN


//#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCCHECK_PROC(proc,fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop2(proc,temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK_PROC(proc,fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){warn_trace(proc,temp_rc);}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
//cIMU    IMU;


// add by nishi for time
const int timeout_ms = 1000;
static int64_t time_ms,time_ns;
static time_t time_seconds;

// for foxbot_core3
//float linear_velocity_ref;
//float angular_velocity_ref;

#include "turtlebot3_sensor.h"
#include "com_lib.h"

#include <Timer.h>
#include <MedianFilter.h>

#include <time.h>

#define FIRMWARE_VER "1.2.6"


/* Global parameters */
//----- moter drive controll ----
//#define FREQUENCY_RATE 					30 			// [ms] default 50ms
//#define FREQUENCY_RATE 						20 			// [ms] default 50ms
#define FREQUENCY_RATE_HZ 					50 			// 20[ms] -> 50[Hz]

//----- rate controler -----
//#define FREQUENCY_CONTROLLER 				30 			// [ms] default 50ms
#define FREQUENCY_CONTROLLER 				20 			// [ms] default 50ms
#define FREQUENCY_CONTROLLER_HZ				50 			// 20[ms] -> 50[Hz]

//---- odometry and tf publish rate ----
#define FREQUENCY_ODOMETRY_HZ             15.0d   // 15[hz]  Act ??[hz]
//#define FREQUENCY_ODOMETRY_HZ             12.0d   // 12[hz]  Act ??[hz] changed by nhishi 2023.3.3

//---- ros spin rate
//#define FREQUENCY_ROSPINONCE_HZ				50		// 50[Hz] -> 20000[us]
#define FREQUENCY_ROSPINONCE_HZ				24		// 24[Hz]  changed by nishi 2023.3.3

//---- imu publish rate -----
//#define FREQUENCY_IMU_PUBLISH_HZ            200  // 5[ms] -> 200[hz]
#define FREQUENCY_IMU_PUBLISH_HZ            15  // 15 [hz]
//#define FREQUENCY_IMU_PUBLISH_HZ            12  // 12 [hz]  changed by nishi 2023.3.3

//---- imu data copy rate ----
#define FREQUENCY_IMU_DATA_HZ               100  // 100[hz]
//#define FREQUENCY_IMU_DATA_HZ               200  // 200[hz]

//---- moter drive parameters ------
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

// add by nishi 2025.3.20
//#define IMU_DATA_RATE_THRES    26       // 28[Hz]
//#define IMU_DATA_RATE_THRES    16       // 18[Hz]
#define IMU_DATA_RATE_THRES    1       // 1[Hz]
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



float yaw_est;
//unsigned long odom_prev_time;
uint64_t odom_prev_time;    // changed by nishi 2022.10.28


/* Prototype function */
void loop_main();
bool create_entities();
void destroy_entities();
void set_my_time(int timeout_ms=1000);

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
//void updateTF(geometry_msgs::TransformStamped& odom_tf);
//void updateTF(geometry_msgs__msg__TransformStamped& odom_tf);
// changed by nishi 2025.3.18
void updateTF(geometry_msgs__msg__TransformStamped& odom_tf,nav_msgs__msg__Odometry& odom);

void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void initOdom(void);
void initJointStates(void);
void updateVariable(bool isConnected);
//ros::Time rosNow();     // add by nishi 2021.7.5
builtin_interfaces__msg__Time rosNow();
void updateGyroCali(bool isConnected); // add by nishi 2021.11.3
void publishImuMsg(void);   // add by nishi 2021.7.5
#if defined(USE_MAG_X)
    void publishMagMsg(void);   // add by nishi 2021.11.4
#endif

void update_motor(void *pvParameters);

float q_prev[4];
double q_Zero[4];
// add by nishi end

/* Velocity command subscriber */
// callback function prototype
void commandVelocityCallback(const void *cmd_vel_msg);
void pc_beatCallback(const void *pc_beat_msg);

/*******************************************************************************
* Subscriber
*******************************************************************************/
// "cmd_vel"
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg0;

#if defined(USE_PC_BEAT)
    // "pc_beat"
    rcl_subscription_t pc_beat_subscriber;
    std_msgs__msg__UInt32 pc_beat_msg0;
#endif

/*******************************************************************************
* Publisher
*******************************************************************************/
// Version information of Turtlebot3
//turtlebot3_msgs::VersionInfo version_info_msg;
//ros::Publisher version_info_pub("firmware_version", &version_info_msg);

/*-------------------------------------------------------
-- IMU publisher
  reffer https://github.com/micro-ROS/micro_ros_arduino/issues/881
--------------------------------------------------------*/
// IMU of Turtlebot3    add by nishi 2021.7.5
//sensor_msgs::Imu imu_msg;

sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t imu_publisher;
const char * imu_topic_name = "imu";
//const char * imu_topic_name = "imu_fox";

//ros::Publisher imu_pub("imu", &imu_msg);

/*-------------------------------------------------------
-- Odometry publisher
--------------------------------------------------------*/
// Odometry of Turtlebot3 ROS2
//nav_msgs__msg__Odometry odom;
// changed by nishi 2025.3.18
nav_msgs__msg__Odometry odom_;
rcl_publisher_t odom_publisher;
#if defined(USE_ODOM_FOX)
    const char * odom_topic_name = "odom_fox";
#else
    const char * odom_topic_name = "odom";
#endif
// type support
//const rosidl_message_type_support_t * odom_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);

// Odometry of Turtlebot3
//nav_msgs::Odometry odom;
//ros::Publisher odom_pub("odom", &odom);
// test by nishi 2021.10.8
//ros::Publisher odom_pub("odom_fox", &odom);

// add by nishi start
// Joint(Dynamixel) state of Turtlebot3
//sensor_msgs::JointState joint_states;
//ros::Publisher joint_states_pub("joint_states", &joint_states);

#if defined(USE_MAG_X)
    // Magnetic field
    sensor_msgs::MagneticField mag_msg;
    ros::Publisher mag_pub("magnetic_field", &mag_msg);
#endif

/*-------------------------------------------------------
-- Transform Broadcaster
--------------------------------------------------------*/

// TF of Turtlebot3 ROS2
rcl_publisher_t tf_publisher;
const char * tf_topic_name = "tf";

tf2_msgs__msg__TFMessage * tf_message;  // 複数 Entry 在るので、memory 上に作成する。

//geometry_msgs::TransformStamped odom_tf;
geometry_msgs__msg__TransformStamped odom_tf;

//tf::TransformBroadcaster tf_broadcaster;

/*-------------------------------------------------------
-- fox_beat publisher   add by nishi 2023.3.6
--------------------------------------------------------*/
#if defined(USE_FOX_BEAT)
    // "fox_beat"
    std_msgs__msg__UInt32 fox_beat;
    rcl_publisher_t fox_beat_publisher;
    const char * fox_beat_topic_name = "fox_beat";
#endif


struct MOTOR_TIC{
    int32_t left;
    int32_t right;
};
MOTOR_TIC motor_tic;


int32_t dzi[3];
int32_t dzi_sum;

bool sen_init=false;

// add by nishi end


/* DEBUG */
std_msgs__msg__UInt32 debug_left;
#if defined(USE_DEBUG)
    //#include <geometry_msgs/msg/point.h>
    //geometry_msgs::Point debug_left;
    //ros::Publisher debug_publisher1("debug_left", &debug_left);

    rcl_publisher_t debug_left_publisher;
    const char * debug_left_topic_name = "debug_left";

    //std_msgs__msg__UInt32 debug_right;
    geometry_msgs__msg__Vector3 debug_right;
    //geometry_msgs__msg__Quaternion debug_right;
    rcl_publisher_t debug_right_publisher;
    const char * debug_right_topic_name = "debug_right";

#endif

template <typename type>
type sign(type value) {
    return type((value>0)-(value<0));
}

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
//unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10]={0,0,0,0,0,0,0,0,0,0};
// [7] : set_my_time() wait time

//uint32_t frequency_odometry_hz;     // FREQUENCY_ODOMETRY_HZ
unsigned long frequency_odometry_hz;     // FREQUENCY_ODOMETRY_HZ
//uint32_t frequency_odometry_hz_ave; // FREQUENCY_ODOMETRY_HZ average
unsigned long frequency_odometry_hz_ave; // FREQUENCY_ODOMETRY_HZ average

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

//----------------------------------------------
// change these params for your robot contorol 2
//
// add by nishi 2022.9.9
// 3) use_tf_static==true : publist tf odom -> base_footprint 
//bool use_tf_static=false;
bool use_tf_static=true;

// 4) use_imu_pub==true : publist 'imu_fox' 
bool use_imu_pub=true;

// 5) use_beat==true : Heart Beat function ON  add by nishi 2023.3.2
// subscribe /fox_beat
bool use_beat=true;

//----------------------------------------------


u_int32_t beat_no=0;
u_int32_t beat_no_prev=0;

bool beat_ok=true;
bool beat_ok_prev=true;

#endif // FOXBOT_CORE_CONFIG_H_

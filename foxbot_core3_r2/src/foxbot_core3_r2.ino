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
 *  Description: Core script for foxbot robot.
 *  Author: Matthieu Magnon
 *
 *  Serial communication is enabled with the following command:
 *  rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
 * update 2021.7.5
 *  Use IMU Sensor (MPU9250).
 * 	turtlebot3_sensor.cpp, turtlebot3_sensor.h, IMU(MPU9250+SPI) were built in.
 */
/*
  esp32 and Micro-ROS Arduino
  https://github.com/micro-ROS/micro_ros_arduino

  system time
  https://qiita.com/BotanicFields/items/f1e28af5a63e4ccf7023

 1. run
  $ sudo chmod 777 /dev/ttyUSB0 or /dev/ttyTHS1
  $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 [-v6] [-b 115200]

 2. check
  $ ros2 topic list
  $ ros2 topic hz /odom_fox

*/

#undef ESP32
#include "foxbot_core3_r2_config.h"

#include <sys/time.h>  // for struct timeval


#if defined(BOARD_ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <analogWrite.h>
#define LED_BUILTIN 17
//#define LED_BUILTIN 4
#endif

#if defined(ESP32)
	#include <WiFi.h>
	//const char SSID[] = "WiFi ID";
	const char SSID[] = "elecom2g-DE45B0";
	//const char PASSWORD[] = "WiFi passsword";
	const char PASSWORD[] = "2389676465277";
	IPAddress server(192,168,1,170);
	const uint16_t serverPort = 11411;
	WiFiClient client;
#endif

#define ODOM_USE_IMU

// Multi task
TaskHandle_t th[2];
SemaphoreHandle_t xMutex = NULL;

int cnt4=0;




const void euler_to_quat(float x, float y, float z, double* q) {
    float c1 = cos((y*3.14/180.0)/2);
    float c2 = cos((z*3.14/180.0)/2);
    float c3 = cos((x*3.14/180.0)/2);

    float s1 = sin((y*3.14/180.0)/2);
    float s2 = sin((z*3.14/180.0)/2);
    float s3 = sin((x*3.14/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

void error_loop(){
  while(1){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

// Int32 cb
//void subscription_callback(const void * msgin)
//{  
//  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
//  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
//}

static time_t epoch_sec_off=0;	// NTP の epcoh sec からのズレ[s] 
static time_t epoch_millis_off=0;	// NTP の epcoh milli sec からのズレ[ms]  50 day over flow

void set_my_time(int timeout_ms){
	//int timeout_ms = 1000;

	struct timeval tv2={0,0};
		//struct timeval {
		//  time_t		tv_sec;		/* seconds */
		//  suseconds_t	tv_usec;	/* and microseconds */
		//};

	// Synchronize time
	if (RCL_RET_OK  != rmw_uros_sync_session(timeout_ms))
		return;
	
	int64_t time_ms = rmw_uros_epoch_millis(); 
    int64_t time_ns = rmw_uros_epoch_nanos();
	unsigned long t = millis();

	if (time_ms > 0){
		//time_seconds = time_ms/1000;
		//setTime(time_seconds); 
		//sprintf(time_str, "%02d.%02d.%04d %02d:%02d:%02d.%03d", day(), month(), year(), hour(), minute(), second(), (uint) time_ms % 1000);

		epoch_sec_off = time_ms/1000 - t/1000;	// millis() : プログラム実行からの経過時間(ms)をtimeに返す
		if((time_ms % 1000) <  (t % 1000)){
			epoch_sec_off--;
			epoch_millis_off = (time_ms % 1000)+1000 - t % 1000;
		}
		else{
			epoch_millis_off = time_ms % 1000 - t % 1000;
		}

		//HWSERIAL.print("Agent date: ");
		//HWSERIAL.println(time_str);  
		
		//struct timeval tv2 = { mktime(&tm_init), 0 };
		//tv2.tv_sec = time_ms/1000;
		//tv2.tv_usec = time_ms % 1000;
		//settimeofday(&tv2, NULL);
	}
	else{
		//HWSERIAL.print("Session sync failed, error code: ");
		//HWSERIAL.println((int) time_ms);  
	}
}

void setup(){
	#if defined(BOARD_ESP32)
	    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
	#endif

	// Builtin LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);		// light OFF
	//digitalWrite(LED_BUILTIN, HIGH);	// light ON

	#if defined(ESP32)
		//Serial.begin(115200);
		//delay(10);
		//Serial.println("start prog.");
	#endif
	//Serial.begin(115200);
	//Serial.begin(1000000);
	//1000000

	#if defined(ESP32X)
		WiFi.begin(SSID,PASSWORD);
		Serial.print("WiFi connecting");

		while (WiFi.status() != WL_CONNECTED) {
			Serial.print(".");
			delay(100);
		}
		Serial.println(" connected");
		Serial.println("IP address: ");
		Serial.println(WiFi.localIP());
	#endif

	// start foxbot_core
	analogWriteResolution(12);
	#if defined(ESP32)
	    //Serial.println("analogWriteResolution(12) ok");
	#endif

	// set Moter control ports
	initMoter();

	#if defined(BOARD_ESP32)
		// add by nishi for ESP32 only
		analogWriteChannel(enMotorRight);
		analogWriteChannel(enMotorLeft);
	#endif

	// https://micro.ros.org/docs/api/rmw/
	// how to change Serial Speed
	// micro_ros_arduino/src/default_transport.cpp
	//  arduino_trans_open() の中で Srerial.begin(speed) をいじる。
    set_microros_transports();

	delay(2000);
	// test by nishi
	//Serial.begin(1000000);

	// alloc TF Message
	tf_message = tf2_msgs__msg__TFMessage__create();
	geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

	tf_message->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
	// header.frame_id
	char string1[] = "/odom";
	memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
	tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
	tf_message->transforms.data[0].header.frame_id.capacity = 100;

	// child_frame_id
	char string2[] = "/base_footprint";
	tf_message->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
	memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
	tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
	tf_message->transforms.data[0].child_frame_id.capacity = 100;


	// Setting for IMU add by nishi 2021.7.5
	sensors.init();
	sen_init=true;

	// Setting for SLAM and navigation (odometry, joint states, TF)
	initOdom();
	initJointStates();
	prev_update_time = micros();
    setup_end = true;
	//Serial.println("setup end");
	delay(100);

	vSemaphoreCreateBinary( xMutex );
	//xMutex = xSemaphoreCreateMutex();

	if( xMutex != NULL ){
		// start IMU Update Task. add by nishi 2021.8.9
		xTaskCreatePinnedToCore(
			update_motor,"update_motor",4096,NULL,1,&th[0],1);
	}
	else {
		while(1){
			Serial.println("rtos mutex create error, stopped");
			delay(1000);
		}
	}

	q_prev[0]=1.0;
	q_prev[1]=0.0;
	q_prev[2]=0.0;
	q_prev[3]=0.0;

	frequency_odometry_hz = (unsigned long)(1000000.0 / FREQUENCY_ODOMETRY_HZ);	// 1 cycle time[micro sec] add by nishi 2022.3.21
	frequency_odometry_hz_ave = frequency_odometry_hz;
	setup_end = true;

  	state = WAITING_AGENT;

}

#ifdef TEST_NOSHI_XXX
void loop_test(){
	#if defined(BOARD_F407VG)
		digitalWrite(LED_RED, HIGH);
	#endif
	//Serial.println("hello");
	// test by nishi
	uint8_t data[] = "hello4\r\n"; 
	nh.getHardware()->write(data,8);
	delay(100);
	#if defined(BOARD_F407VG)
		digitalWrite(LED_RED, LOW);
	#endif
	delay(100);
}
#endif

void loop() {
  states state_prv=state;
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
	  else{
		set_my_time();	// NTP 時刻合わせ

		// set_my_time() wait time
		tTime[7] = foxbot3::micros_() + (5 * 60 * 1000000UL );	// 5[minute]

		updateVariable(true);
		updateTFPrefix(true);
	  }

      break;
    case AGENT_CONNECTED:
      //EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
	  //if(RMW_RET_OK != rmw_uros_ping_agent(100, 1)){
	  if(RMW_RET_OK != rmw_uros_ping_agent(100, 3)){	// changed by nishi 2022.12.5
		state = AGENT_DISCONNECTED;
	  }
      if (state == AGENT_CONNECTED) {
		// 従来の loop() 処理
		loop_main();
		// 受信 処理
        //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

		// syscronize NTP
		foxbot3::usec_t t = foxbot3::micros_();
		// set_my_time() wait time over
		if(t >= tTime[7]){
			set_my_time(50);
			tTime[7] = t + (5 * 60 * 1000000UL );	// 5[minute]
		}
		
      }
      break;
    case AGENT_DISCONNECTED:
	  updateVariable(false);
	  updateTFPrefix(false);
	  // stop motor
	  linear_velocity_ref=0.0;
	  angular_velocity_ref=0.0;

      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if(state != state_prv){
	if (state == AGENT_CONNECTED) {
	  digitalWrite(LED_BUILTIN, LOW);		// light OFF
	} 
	else {
	  digitalWrite(LED_BUILTIN, HIGH);	// light ON
	}
  }
}

void loop_main(){
	//bool imu_exec=false;
	CB cb;
	double dlt[3];

	// add by nishi
	//updateVariable(true);
	//updateTFPrefix(true);

	struct timespec tv;
	// こちらは、データが入ってこない。
	//clock_gettime(0, &tv);
	// esp32 の system mills + NTP epoch millis off -> Now mills
	unsigned long u_ms = millis();
	tv.tv_sec = u_ms/1000 + epoch_sec_off;		// epoch sec
	unsigned long ms = u_ms%1000 + epoch_millis_off;
	if(ms > 1000 )
	{
		tv.tv_sec++;
		ms -=1000;
	}
	tv.tv_nsec = ms*1000*1000;	// epoch nano sec


	// Start Gyro Calibration after ROS connection  add by nishi 2021.11.3
	//updateGyroCali(nh.connected());

	//sensors.updateIMU();

	//uint32_t t = millis();
	//unsigned long t = micros();
	foxbot3::usec_t t = foxbot3::micros_();

	// update odometry
	if (t >= tTime[2]){
		//float dt, dx, dy, dz;
		double dt, dx, dy, dz;
		//float qw, qx, qy, qz;
		//double qw, qx, qy, qz;
		double q[4];
		double q_tmp[4];
		double roll, pitch, yaw;

		//dt = (float)(millis() - odom_prev_time) * 0.001f;
		//dt = (double)(micros() - odom_prev_time) * 0.000001;	// dt[Sec]
		dt = (double)(t - odom_prev_time) * 0.000001;	// dt[Sec]
		//odom_prev_time = millis();
		//odom_prev_time = micros();
		odom_prev_time = t;

		// compute linear and angular estimated velocity
		linear_velocity_est = WHEEL_RADIUS * (motor_right_rate_est + motor_left_rate_est) / 2.0f;
		angular_velocity_est = (WHEEL_RADIUS / BASE_LENGTH)
							 * (motor_right_rate_est - motor_left_rate_est);

		// compute translation and rotation
		yaw_est += angular_velocity_est * dt;
		dx = cos(yaw_est) * linear_velocity_est * dt;	// 基本座標系
		dy = sin(yaw_est) * linear_velocity_est * dt;	// 基本座標系
		dz = 0.0;

		// DEBUG
		//debug_left.y = yaw_est * 57.2958;
		//debug_left.z = angular_velocity_est * dt;
		//debug_publisher1.publish(&debug_left);

		#ifndef ODOM_USE_IMU
			// compute quaternion
			q[0] = cos(abs(yaw_est) / 2.0f);		// qw
			q[1] = 0.0f;							// qx
			q[2] = 0.0f;							// qy
			q[3] = sign(yaw_est) * sin(abs(yaw_est) / 2.0f);	// gz
		#else
			bool ok =true;
			if(pdTRUE == xSemaphoreTake(xMutex, 10UL)){
				ok=false;
			}
			// get IMU data
			imu_msg = sensors.getIMU();
			if(ok){
				xSemaphoreGive(xMutex);
			}
			q[0] = imu_msg.orientation.w;
			q[1] = imu_msg.orientation.x;
			//q[2] = imu_msg.orientation.y - 0.013;	// ロボットの前傾を調整。 adjust verical angle IMU
			q[2] = imu_msg.orientation.y;	// ロボットの前傾 そのまま。 2022.10.28
			q[3] = imu_msg.orientation.z;

			//SERIAL_PORT.print(F("q[0]:"));
			//SERIAL_PORT.print(q[0], 3);
			//SERIAL_PORT.print(F(" q[1]:"));
			//SERIAL_PORT.print(q[1], 3);
			//SERIAL_PORT.print(F(" g[2]:"));
			//SERIAL_PORT.print(q[2], 3);
			//SERIAL_PORT.print(F(" q[3]:"));
			//SERIAL_PORT.println(q[3], 3);


			// 1つ前のとの 4:6 のクォータニオンをつかいます。
			//q_tmp[1] = (float)(q[1]*0.3 + q_prev[1]*0.7);
			//q_tmp[2] = (float)(q[2]*0.3 + q_prev[2]*0.7);
			//q_tmp[3] = (float)(q[3]*0.3 + q_prev[3]*0.7);

			//q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W
			//q_tmp[0] = sqrt(1.0 - ((q_tmp[1] * q_tmp[1]) + (q_tmp[2] * q_tmp[2]) + (q_tmp[3] * q_tmp[3])));

			q[0]=1.0 - ((q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
			if(q[0] >= 0.0) q[0] = sqrt(q[0]);
			else q[0] = sqrt(q[0] * -1.0) * -1.0;


			//sensors.imu_.compCB(q_tmp,&cb);
			sensors.imu_.compCBd(q,&cb);

			//q_prev[0]=q[0];
			//q_prev[1]=q[1];
			//q_prev[2]=q[2];
			//q_prev[3]=q[3];
			double dx1,dy1,dz1;

			#if defined(TEST_10_x3)
			cnt4++;
			if (cnt4==0)
				linear_velocity_est=0.0;
			else if(cnt4==1)
				linear_velocity_est=0.005;
			else if(cnt4==2)
				linear_velocity_est=0.002;
			else if(cnt4==3)
				linear_velocity_est=-0.002;
			else if(cnt4==4)
				linear_velocity_est=0.0;
			else{
				linear_velocity_est=-0.005;
				cnt4=0;
			}
			#endif

			dx1 = linear_velocity_est * dt;
			dy1 = 0.0;
			dz1=0.0;

			dlt[0]=cb.dt[0][0]*dx1+cb.dt[0][1]*dy1+cb.dt[0][2]*dz1;
			dlt[1]=cb.dt[1][0]*dx1+cb.dt[1][1]*dy1+cb.dt[1][2]*dz1;
			dlt[2]=cb.dt[2][0]*dx1+cb.dt[2][1]*dy1+cb.dt[2][2]*dz1;


			dx = dlt[0];
			dy = dlt[1];
			dz = dlt[2];

			//if (fabs(dx) < 0.00025){
			//	dz=0.0;
			//}

			//#define TEST_10_x
			#if defined(TEST_10_x)
			SERIAL_PORT.print(F("dx:"));
			SERIAL_PORT.print(dx, 6);
			SERIAL_PORT.print(F(" dy:"));
			SERIAL_PORT.print(dy, 6);
			SERIAL_PORT.print(F(" dz:"));
			SERIAL_PORT.println(dz, 6);
			#endif

		#endif

		// feed odom message
		//odom.header.stamp = nh.now();
		odom.header.stamp.sec = tv.tv_sec;
		odom.header.stamp.nanosec = tv.tv_nsec;
		updateOdometry();
		//odom.header.frame_id = "odom";
		//odom.child_frame_id = "base_footprint";
		odom.pose.pose.position.x += dx;
		odom.pose.pose.position.y += dy;
		//odom.pose.pose.position.z = 0.0;
		odom.pose.pose.position.z += dz;
		odom.pose.pose.orientation.w = q[0];
		odom.pose.pose.orientation.x = q[1];
		odom.pose.pose.orientation.y = q[2];
		odom.pose.pose.orientation.z = q[3];
		// Velocity expressed in base_link frame
		odom.twist.twist.linear.x = linear_velocity_est;
		odom.twist.twist.linear.y = 0.0f;
		odom.twist.twist.angular.z = angular_velocity_est;

		//#define TEST_10_x1
		#if defined(TEST_10_x1)
			SERIAL_PORT.print(F("x:"));
			SERIAL_PORT.print(odom.pose.pose.position.x*10000.0, 8);
			SERIAL_PORT.print(F(" y"));
			SERIAL_PORT.print(odom.pose.pose.position.y*10000.0, 8);
			SERIAL_PORT.print(F(" z:"));
			SERIAL_PORT.println(odom.pose.pose.position.z*10000.0, 8);
		#endif

		// error occured -> [ERROR] [1616575654.217167]: Message from device dropped: message larger than buffer.  by nishi
		//odom.header.stamp = nh.now();
    	//odom_pub.publish(&odom);
		RCSOFTCHECK(rcl_publish(&odom_publisher, &odom, NULL));

		// add by nishi 2022.9.9
		// use_tf_static==true -> publist tf base_footprint 
		if(use_tf_static==true){
			// add by nishi for TF  2021.4.26
			// odometry tf
			//updateTF(odom_tf);
			updateTF(tf_message->transforms.data[0]);
			//odom_tf.header.stamp = nh.now();
			//odom_tf.header.stamp = odom.header.stamp;

			// set time stamp
		    tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
			//tf_message->transforms.data[0].header.stamp.nanosec = time_ns;
		    tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;
			//tf_message->transforms.data[0].header.stamp.sec = time_seconds;

			// ratbmap-nishi_stereo_outdoor.launch と TF がバッテイングする? 2021.9.16
			RCSOFTCHECK(rcl_publish(&tf_publisher, tf_message, NULL));
		}
		
		//delay(1);

		// add by nishi for joint states pub. but not yet correct work
		int32_t left_tick=motor_tic.left;
		int32_t right_tick=motor_tic.right;
		//motor_tic.left=0;
		//motor_tic.right=0;
		updateMotorInfo(left_tick,right_tick);
		// joint states
		updateJointStates();
		//joint_states.header.stamp = nh.now();
		//joint_states.header.stamp = odom.header.stamp;

		//joint_states_pub.publish(&joint_states);

		//Serial.print("motor_tic.left=");
		//Serial.print(motor_tic.left, DEC);
		//Serial.println("");
		//Serial.print("motor_tic.right=");
		//Serial.print(motor_tic.right, DEC);
		//Serial.println("");


		tTime[2] = t + frequency_odometry_hz;	// set 1-cycle-time [ms] to odom Timer.

		t = foxbot3::micros_();
    }

	// Update the IMU unit.	add by nishi 2021.7.5
	//if(imu_exec == false){
	//	sensors.updateIMU();
	//	imu_exec=true;
	//}

	// IMU Publish.	add by nishi 2021.7.5
	//if(_frequency_imu.delay(millis())) {
	if (t >= tTime[3]){
		if(use_imu_pub==true){
	    	publishImuMsg();
		}
		#ifdef USE_MAG
		    publishMagMsg();
		#endif
	    tTime[3] = t + (1000000UL / FREQUENCY_IMU_PUBLISH_HZ);
		//t = millis();		// comment in by nishi 2022.3.18
	}

	//if(_frequency_ver_info.delay(millis())) {
	//	publishVersionInfoMsg();
	//}

	// update subscribers values
	if (t >= tTime[5]){
		// Subscribe topic の 受信時に必要
		//RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));

	    //tTime[5] = t + 2000;	// wait 2[ms]
	    tTime[5] = t + 20000;	// wait 20[ms]
	}

}


//twist message cb
void commandVelocityCallback(const void *cmd_vel_msg){
	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)cmd_vel_msg;
	// if velocity in x direction is 0 turn off LED, if 1 turn on LED
	//digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);

	linear_velocity_ref  = ((const geometry_msgs__msg__Twist *)cmd_vel_msg)->linear.x;
	angular_velocity_ref = ((const geometry_msgs__msg__Twist *)cmd_vel_msg)->angular.z;
}

bool create_entities()
{
	allocator = rcl_get_default_allocator();

	//create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	#ifdef USE_ORG1
		// create node
		RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
	#else
		// create node
		// reffer from  https://qiita.com/ousagi_sama/items/a814b3db925b7ce2aeea
		// ノードの生成 (foxy版)
		rcl_node_options_t node_ops = rcl_node_get_default_options();
		node_ops.domain_id = (size_t)(30);		// ドメインIDの設定
		//rclc_node_init_with_options(&node, "node_name", "namespace", &support, &node_ops);
		rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops);

		// ノードの生成 (galactic版以降)
		//rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
		//RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
		//RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 30));		// ドメインIDの設定
		//rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator); // 前のrclc_support_initは削除する
		//rclc_node_init_default(&node, "node_name", "namespace", &support);

	#endif

	// craete publisher for "odom_fox"
	RCCHECK(rclc_publisher_init_default(
		&odom_publisher, &node,
		//odom_type_support,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), 
		odom_topic_name));	

	// create publisher for "tf"
	RCCHECK(rclc_publisher_init_default(
		&tf_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
		tf_topic_name));

	// create subscriber Int32
	//RCCHECK(rclc_subscription_init_default(
	//  &subscriber,
	//  &node,
	//  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	//  "micro_ros_arduino_subscriber"));

	// create subscriber for Twist "cmd_vel"
	RCCHECK(rclc_subscription_init_default(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// create timer,
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
	&timer,
	&support,
	RCL_MS_TO_NS(timer_timeout),
	timer_callback));

	// create executor
	//  for publisher
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	//  for cmd_vel subscriber
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg0, &commandVelocityCallback, ON_NEW_DATA));

	return true;
}

void destroy_entities()
{
  rcl_ret_t rc;
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rc=rcl_publisher_fini(&odom_publisher, &node);
  rc=rcl_publisher_fini(&tf_publisher, &node);
  rc=rcl_subscription_fini(&cmd_vel_subscriber, &node);

  rc=rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rc=rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void update_motor(void *pvParameters){
	xSemaphoreGive(xMutex);

	int ac_cnt2=0;
	clock_t t_start2 ,t_end2;

	while(1){
		//uint32_t t = millis();
		//unsigned long t = micros();
		foxbot3::usec_t t = foxbot3::micros_();	// update by nishi 2022.10.28

		// rate computation
		//if(_frequency_rate.delay(millis())) {
		if (t >= tTime[0]){

			//digitalWrite(RT_PIN0, HIGH);

			//float dt;
			double dt;

			motor_tic.left += motor_left_direction*motor_left_inc;	// add by nishi for joint state
			motor_tic.right += motor_right_direction*motor_right_inc;	// add by nishi for joint state

			// MOTOR RIGHT
			// direction
			motor_right_direction_median_filter.in(motor_right_direction);
			motor_right_filtered_direction = motor_right_direction_median_filter.out();

			// filter increment per second
			dt = (micros() - motor_right_prev_time);
			motor_right_prev_time = micros();
			motor_right_filtered_inc_per_second = runningAverage(motor_right_filtered_inc_per_second,
					(float)motor_right_inc / dt * 1000000.0f, RATE_AVERAGE_FILTER_SIZE);

			// estimated rate
			motor_right_rate_est = (float)motor_right_filtered_direction
									* motor_right_filtered_inc_per_second * RATE_CONV;

			motor_right_inc = 0;

			if (abs(motor_right_rate_est) < 0.1f)
				motor_right_rate_est = 0.0f;

			motor_right_check_dir = 1;
			motor_right_direction = 0;

			// MOTOR LEFT
			// direction
			motor_left_direction_median_filter.in(motor_left_direction);
			motor_left_filtered_direction = motor_left_direction_median_filter.out();

			// filter increment per second
			dt = (micros() - motor_left_prev_time);
			motor_left_prev_time = micros();
			motor_left_filtered_inc_per_second = runningAverage(motor_left_filtered_inc_per_second,
					(float)motor_left_inc / dt * 1000000.0f, RATE_AVERAGE_FILTER_SIZE);

			// estimated rate
			motor_left_rate_est = (float)motor_left_filtered_direction
								* motor_left_filtered_inc_per_second * RATE_CONV;
			motor_left_inc = 0;

			if (abs(motor_left_rate_est) < 0.1f)
				motor_left_rate_est = 0.0f;
			motor_left_check_dir = 1;
			motor_left_direction = 0;

			// MIXER
			motor_right_rate_ref = (linear_velocity_ref + BASE_LENGTH / 2.f * angular_velocity_ref)
									/ (WHEEL_RADIUS);
			motor_left_rate_ref  = (linear_velocity_ref - BASE_LENGTH / 2.f * angular_velocity_ref)
									/ (WHEEL_RADIUS);
			//digitalWrite(RT_PIN0, LOW);

			tTime[0] = t + (1000000UL / FREQUENCY_RATE_HZ);
			t = foxbot3::micros_();
		}

		// rate controler
		//if(_frequency_controller.delay(millis())) {
		if (t >= tTime[1]){
			//digitalWrite(RT_PIN1, HIGH);

			// MOTOR RIGHT
			rateControler(motor_right_rate_ref, motor_right_rate_est, motor_right_pwm_rate,
							controler_motor_right_prev_time, controler_motor_right_prev_epsilon,
							controler_motor_right_int);
			pwmMotorRight = pwmMotorRight + motor_right_pwm_rate;
			pwmMotorRight = constrain(pwmMotorRight, 0, PWM_MAX);
			setMotorRateAndDirection(pwmMotorRight, motor_right_rate_ref, enMotorRight,
										in1MotorRight, in2MotorRight);

			// MOTOR LEFT
			rateControler(motor_left_rate_ref, motor_left_rate_est, motor_left_pwm_rate,
							controler_motor_left_prev_time, controler_motor_left_prev_epsilon,
							controler_motor_left_int);
			pwmMotorLeft = pwmMotorLeft + motor_left_pwm_rate;
			pwmMotorLeft = constrain(pwmMotorLeft, 0, PWM_MAX);
			setMotorRateAndDirection(pwmMotorLeft, motor_left_rate_ref, enMotorLeft,
										in1MotorLeft, in2MotorLeft);

			// DEBUG
			//debug_left.x = motor_left_rate_ref;
			//debug_left.y = motor_left_rate_est;
			// add by nishi 2021.4.24
			//debug_left.z = pwmMotorLeft;

			//debug_publisher1.publish(&debug_left);

			// DEBUG
			//debug_right.x = motor_right_rate_ref;
			//debug_right.y = motor_right_rate_est;
			//debug_right.z = pwmMotorRight;
			//debug_publisher2.publish(&debug_right);
			//digitalWrite(RT_PIN1, LOW);

			tTime[1] = t + (1000000UL / FREQUENCY_CONTROLLER_HZ);
			t = foxbot3::micros_();		// add by nishi 2022.3.18
		}

		if (t >= tTime[4]){
			//Serial.println("start update_imu()");
			// Update the IMU unit.	add by nishi 2021.8.9
			//sensors.updateIMU();
			if(sen_init==true){
				bool ok =true;
				if(pdTRUE != xSemaphoreTake(xMutex,10UL)){
					ok=false;
				}
				sensors.copyIMU();
				if(ok){
					xSemaphoreGive(xMutex);
				}
			}
			tTime[4] = t + (1000000UL / FREQUENCY_IMU_DATA_HZ);
		}
	    if (t >= tTime[6]){
			if(sen_init==true){
				sensors.updateIMU();
			}
			//                                    無負荷時  /  Madgwick時 / DMP6時  / DMP6 + Acc
			//tTime[6] = t + 1000000UL / 100;   //    
			//tTime[6] = t + 1000000UL / 250;   //    
			//tTime[6] = t + 1000000UL / 400;   //        /  206        /   205     /
			//tTime[6] = t + 1000000UL / 450;   //        /  216        /        /
			//tTime[6] = t + 1000000UL / 600;   //    
			tTime[6] = t + 1000000UL / 700;   //        /  263        /   300     /
			//tTime[6] = t + 1000000UL / 950;   //        /  292        /        /
			//tTime[6] = t + 1000000UL / 1000;  //        /  300        /   338   /  338 だけど tf が変
			// 無制限 							 //        / 1544 - 1620 / 3700   / 1600 だけど tf が変
			ac_cnt2++;
		}

		//#define XXX_400
		#if defined(XXX_400)
			t_end2=clock();
			double t_d = (double)(t_end2 - t_start2) / CLOCKS_PER_SEC;
			if(t_d >= 4.0){
				float hz = (float)ac_cnt2/t_d;
				SERIAL_PORT.print(F("acc_hz:"));
				SERIAL_PORT.println(hz, 4);
				t_start2=clock();
				ac_cnt2=0;
			}
		#endif

		//delay(1);
		//vTaskDelay(1);
	}
}

void motorRightIsrCounterDirection() {
	motor_right_inc ++;
	if ( motor_right_check_dir == 1) {
		if (digitalRead(motorRightEncoderPinB) && digitalRead(motorRightEncoderPinA)){
			motor_right_direction = 1;
		} else {
			motor_right_direction = -1;
		}
		//motor_right_check_dir = 1;
		// update by nishi
		motor_right_check_dir = 0;
	}
}

void motorLeftIsrCounterDirection() {
	motor_left_inc ++;
	if ( motor_left_check_dir == 1) {
		if ( digitalRead(motorLeftEncoderPinB) && digitalRead(motorLeftEncoderPinA)){
			motor_left_direction = 1;
		} else {
			motor_left_direction = -1;
		}
		motor_left_check_dir = 0;
	}
}

void rateControler(const float rate_ref, const float rate_est, int & pwm_rate,
				   unsigned long & prev_time, float & previous_epsilon, float & i_epsilon) {

	float epsilon = abs(rate_ref) - abs(rate_est);
	float d_epsilon = (epsilon - previous_epsilon) / (prev_time - millis());

	// reset and clamp integral (todo : add anti windup)
	if (rate_ref == 0.0) {
		i_epsilon = 0.0;
	} else {
		i_epsilon += epsilon * (prev_time - millis()) * RATE_CONTROLLER_KI;
	}
	i_epsilon = constrain(i_epsilon, -RATE_INTEGRAL_FREEZE, RATE_INTEGRAL_FREEZE);

	prev_time = millis();
	previous_epsilon = epsilon;

	//debug_left.z = i_epsilon * RATE_CONTROLLER_KI;

	pwm_rate = epsilon * RATE_CONTROLLER_KP
			 + d_epsilon * RATE_CONTROLLER_KD
			 + i_epsilon * RATE_CONTROLLER_KI;

	// saturate output
	pwm_rate = constrain(pwm_rate, RATE_CONTROLLER_MIN_PWM, RATE_CONTROLLER_MAX_PWM);
}

float runningAverage(float prev_avg, const float val, const int n) {
	return (prev_avg * (n - 1) + val) / n;
}

void setMotorRateAndDirection(int pwm_ref, const float rate_ref,
							  const byte enMotor, const byte in1Motor, const byte in2Motor) {

		// avoid noisy pwm range
		if (abs(rate_ref) < 0.1)
			pwm_ref = 0;

		// write direction
		if (rate_ref > 0) {
			digitalWrite(in1Motor, LOW);
			digitalWrite(in2Motor, HIGH);
		}
		else if (rate_ref < 0) {
			digitalWrite(in1Motor, HIGH);
			digitalWrite(in2Motor, LOW);
		}

		// write pwm
		#if defined(BOARD_ESP32)
		analogWrite(enMotor, pwm_ref,PWM_MAX);
		#else
		analogWrite(enMotor, pwm_ref);
		#endif
}
// add by nishi
/*******************************************************************************
* initMoter
*******************************************************************************/
void initMoter(void){
	// define pin mode motor Right
	//pinMode(motorRightEncoderPinA, INPUT);
	pinMode(motorRightEncoderPinA, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(motorRightEncoderPinA),
					motorRightIsrCounterDirection, RISING);
	//pinMode(motorRightEncoderPinB, INPUT);
	pinMode(motorRightEncoderPinB, INPUT_PULLUP);
	pinMode(enMotorRight, OUTPUT);
	pinMode(in1MotorRight, OUTPUT);
	pinMode(in2MotorRight, OUTPUT);		// PIN Number change for board

	// define pin mode motor Left
	//pinMode(motorLeftEncoderPinA, INPUT);
	pinMode(motorLeftEncoderPinA, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(motorLeftEncoderPinA),
					motorLeftIsrCounterDirection, RISING);
	//pinMode(motorLeftEncoderPinB, INPUT);
	pinMode(motorLeftEncoderPinB, INPUT_PULLUP);
	pinMode(enMotorLeft, OUTPUT);
	pinMode(in1MotorLeft, OUTPUT);
	pinMode(in2MotorLeft, OUTPUT);	// here
}
/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
//ros::Time rosNow()
builtin_interfaces__msg__Time rosNow()
{
	builtin_interfaces__msg__Time t;
  //return nh.now();
  return t;
}


/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  #ifdef USE_ROS1
  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
  #endif
}
/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();		// Now dummy
  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

  //imu_pub.publish(&imu_msg);
}

#ifdef USE_MAG
	/*******************************************************************************
	* Publish msgs (Magnetic data)
	*******************************************************************************/
	void publishMagMsg(void)
	{
	mag_msg = sensors.getMag();

	mag_msg.header.stamp    = rosNow();
	mag_msg.header.frame_id = mag_frame_id;

	mag_pub.publish(&mag_msg);
	}
#endif

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  #ifdef USE_ROS1
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
  #endif
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
	  // commment out ? by nishi 2021.10.55
	  sen_init=false;
	  for(int i=0;i<4;i++){
      	if (0==sensors.initIMU()){
	      sen_init=true;
          break;
		}
	    delay(1000);        // delay mill sec 1000 [ms]
	  }

      initOdom();

	  dzi[0]=0;
	  dzi[1]=0;
	  dzi[2]=0;

	  q_prev[0]=1.0;
	  q_prev[1]=0.0;
	  q_prev[2]=0.0;
	  q_prev[3]=0.0;

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  //char log_msg[50];
  char log_msg[60];

  if (isConnected)
  {
    if (isChecked == false)
    {
      //nh.getParam("~tf_prefix", &get_tf_prefix);
	  get_tf_prefix[0]=0x00;
      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  
        //sprintf(odom_child_frame_id, "base_fox");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");
        //strcat(odom_child_frame_id, "/base_fox");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      //nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      //nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      //nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      //nh.loginfo(log_msg); 

	  // add by nishi 2022.9.9
	  // use_tf_static==true -> publist tf base_footprint 
      use_tf_static=true;
	  bool yes_no=true;
      //nh.getParam("~use_tf_static", &yes_no);
      if (yes_no == false){
		use_tf_static=false;
		sprintf(log_msg, "Setup use_tf_static [false]");
		//nh.loginfo(log_msg); 
	  }
	  else{
		sprintf(log_msg, "Setup use_tf_static [true]");
		//nh.loginfo(log_msg); 
	  }

	  use_imu_pub=false;
	  yes_no=false;
      //nh.getParam("~use_imu_pub", &yes_no);
      if (yes_no == true){
		use_imu_pub=true;
		sprintf(log_msg, "Setup use_imu_pub [true]");
		//nh.loginfo(log_msg); 
	  }
	  else{
		sprintf(log_msg, "Setup use_imu_pub [false]");
		//nh.loginfo(log_msg); 
	  }

      isChecked = true;

    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id.data = odom_header_frame_id;
  // 此処は、後で見直し。 by nishi 2022.10.23
  odom.header.frame_id.size = strlen(odom_header_frame_id);
  odom.header.frame_id.capacity = sizeof(odom_header_frame_id);
  odom.child_frame_id.data  = odom_child_frame_id;
  // 此処は、後で見直し。 by nishi 2022.10.23
  odom.child_frame_id.size = strlen(odom_child_frame_id);
  odom.child_frame_id.capacity = sizeof(odom_child_frame_id);

  //odom.pose.pose.position.x = odom_pose[0];
  //odom.pose.pose.position.y = odom_pose[1];
  //odom.pose.pose.position.z = 0;
  //odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  //odom.twist.twist.linear.x  = odom_vel[0];
  //odom.twist.twist.angular.z = odom_vel[2];

}
/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  #ifdef USE_ROS1
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;

  //Serial.println("updateJointState() : #9 passed!!");
  //Serial.println(*joint_states.position,DEC);
  //Serial.println(*(joint_states.position+1),DEC);
  #endif

}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
//void updateTF(geometry_msgs::TransformStamped& odom_tf)
void updateTF(geometry_msgs__msg__TransformStamped& odom_tf)
{
  //odom_tf.header = odom.header;	// odom or odom_fox
  //odom_tf.child_frame_id = odom.child_frame_id;	// base_footprint
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}


/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  #ifdef USE_ROS1
  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;

  motor_tic.left=0;		// add by nishi
  motor_tic.right=0;	// add by nishi

  yaw_est=0.;			// add by nishi
  #endif

}
/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  #ifdef USE_ROS1
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  //joint_states.effort_length   = WHEEL_NUM;
  // update by nishi 2021.5.10
  joint_states.effort_length   = 0;
  #endif

}

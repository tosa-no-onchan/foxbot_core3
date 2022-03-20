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
esp32 rosserial 
https://gist.github.com/KobayashiRui/094ac01d9d3cd2445faa2a1ef103646f
*/

#undef ESP32
#include "foxbot_core_config.h"

#if defined(BOARD_ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <analogWrite.h>
#define LED_BUILTIN 17
//#define LED_BUILTIN 4
#endif

#if defined(ESP32)
#include <WiFi.h>
const char SSID[] = "WiFi ID";
const char PASSWORD[] = "WiFi passsword";
IPAddress server(192,168,1,170);
const uint16_t serverPort = 11411;
WiFiClient client;
#endif

#define ODOM_USE_IMU

// Multi task
TaskHandle_t th[2];
SemaphoreHandle_t xMutex = NULL;

void update_motor(void *pvParameters){
	xSemaphoreGive(xMutex);
	while(1){
		uint32_t t = millis();

		// rate computation
		//if(_frequency_rate.delay(millis())) {
		if (t >= tTime[0]){

			//digitalWrite(RT_PIN0, HIGH);

			float dt;

			motor_tic.left += motor_left_direction*motor_left_inc;	// add by nishi for joint state
			motor_tic.right += motor_right_direction*motor_right_inc;	// add by nishi for joint state

			// MOTOR RIGHT
			// direction
			motor_right_direction_median_filter.in(motor_right_direction);
			motor_right_filtered_direction = motor_right_direction_median_filter.out();

			// filter increment per second
			dt = (millis() - motor_right_prev_time);
			motor_right_prev_time = millis();
			motor_right_filtered_inc_per_second = runningAverage(motor_right_filtered_inc_per_second,
					(float)motor_right_inc / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);

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
			dt = (millis() - motor_left_prev_time);
			motor_left_prev_time = millis();
			motor_left_filtered_inc_per_second = runningAverage(motor_left_filtered_inc_per_second,
					(float)motor_left_inc / dt * 1000.0f, RATE_AVERAGE_FILTER_SIZE);

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

			tTime[0] = t + (1000 / FREQUENCY_RATE_HZ);
			t = millis();
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

			tTime[1] = t + (1000 / FREQUENCY_CONTROLLER_HZ);
			t = millis();		// add by nishi 2022.3.18
		}

		if (t >= tTime[4]){

			//Serial.println("start update_imu()");
			// Update the IMU unit.	add by nishi 2021.8.9
			//sensors.updateIMU();
			bool ok =true;
			if(pdTRUE != xSemaphoreTake(xMutex,10UL)){
				ok=false;
			}
			sensors.copyIMU();
			if(ok){
				xSemaphoreGive(xMutex);
			}
			tTime[4] = t + (1000 / FREQUENCY_IMU_DATA_HZ);
		}
		sensors.updateIMU();
		//delay(1);
		//vTaskDelay(1);
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
	Serial.begin(115200);
    delay(10);
    Serial.println("start prog.");
	#endif

	#if defined(ESP32)
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
    Serial.println("analogWriteResolution(12) ok");
	#endif

	// set Moter control ports
	initMoter();

	#if defined(BOARD_ESP32)
	// add by nishi for ESP32 only
	analogWriteChannel(enMotorRight);
	analogWriteChannel(enMotorLeft);
	#endif

	#if defined(ESP32)
	Serial.println("nh.getHardware()->setConnection(() call");
	delay(100);
	// set Server IP & port  ESP32 Wi-FI add by nishi 2021.5.6
	nh.getHardware()->setConnection(server, serverPort);
	#else
	// set serial spped
	//nh.getHardware()->setBaud(115200);
	//nh.getHardware()->setBaud(230400);
	//nh.getHardware()->setBaud(250000);
	//nh.getHardware()->setBaud(500000);
	//nh.getHardware()->setBaud(1000000);
	nh.getHardware()->setBaud(2000000);
	#endif

	#if defined(ESP32)
	Serial.println("nh.initNode() call");
	#endif
	delay(100);
	// ROS node initialization
	nh.initNode();


	#if defined(ESP32)
	IPAddress ipa= nh.getHardware()->getLocalIP();
	Serial.print("ip:");
	Serial.println(ipa.toString());
	#endif

	// Subscriber
	nh.subscribe(cmd_vel_sub);

	#if defined(CAMERA_SYNC) or defined(CAMERA_SYNC_EX)
	delay(100);
	camera_info_f = false;
	nh.subscribe(camera_sync_sub);
	delay(100);
	#endif

	// Publisher
    //nh.advertise(version_info_pub);
	nh.advertise(imu_pub);		// add by nishi 2021.7.5
	nh.advertise(odom_pub);
	// add by nishi  2021.4.26
	nh.advertise(joint_states_pub);

	#ifdef USE_MAG
	nh.advertise(mag_pub);
	#endif

	// DEBUG
	nh.advertise(debug_publisher1);
	nh.advertise(debug_publisher2);

	// call ros::Publisher() for "/tf"
	tf_broadcaster.init(nh);

	// RT
	//pinMode(RT_PIN0, OUTPUT);
	//pinMode(RT_PIN1, OUTPUT);

	// Setting for IMU add by nishi 2021.7.5
	sensors.init();

	// Setting for SLAM and navigation (odometry, joint states, TF)
	initOdom();
	initJointStates();
	prev_update_time = millis();
    setup_end = true;
	//Serial.println("setup end");
	delay(100);

	// define tasks frequency
	//_frequency_rate.start((unsigned long) millis());
	//_frequency_controller.start((unsigned long) millis());
	//_frequency_odometry.start((unsigned long) millis());
	//_frequency_rospinonce.start((unsigned long) millis());
	//_frequency_imu.start((unsigned long) millis());		// add by nishi 2021.7.5

	xMutex = xSemaphoreCreateMutex();

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

	frequency_odometry_hz = (uint32_t)(1000.0f / FREQUENCY_ODOMETRY_HZ);	// 1 cycle time[ms] add by nishi 2022.3.5
	frequency_odometry_hz_ave = frequency_odometry_hz;
	setup_end = true;
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

void loop(){
	//bool imu_exec=false;

	// add by nishi
	updateVariable(nh.connected());
	updateTFPrefix(nh.connected());

	// Start Gyro Calibration after ROS connection  add by nishi 2021.11.3
	//updateGyroCali(nh.connected());

	//sensors.updateIMU();

	uint32_t t = millis();

	// update odometry
	if (t >= tTime[2]){
		float dt, dx, dy, dz;
		float qw, qx, qy, qz;
		double roll, pitch, yaw;

		dt = (float)(millis() - odom_prev_time) * 0.001f;
		odom_prev_time = millis();

		// compute linear and angular estimated velocity
		linear_velocity_est = WHEEL_RADIUS * (motor_right_rate_est + motor_left_rate_est) / 2.0f;
		angular_velocity_est = (WHEEL_RADIUS / BASE_LENGTH)
							 * (motor_right_rate_est - motor_left_rate_est);

		// compute translation and rotation
		yaw_est += angular_velocity_est * dt;
		dx = cos(yaw_est) * linear_velocity_est * dt;
		dy = sin(yaw_est) * linear_velocity_est * dt;
		dz = 0.0;

		// DEBUG
		debug_left.y = yaw_est * 57.2958;
		debug_left.z = angular_velocity_est * dt;
		debug_publisher1.publish(&debug_left);

		#ifndef ODOM_USE_IMU
		// compute quaternion
		qw = cos(abs(yaw_est) / 2.0f);
		qx = 0.0f;
		qy = 0.0f;
		qz = sign(yaw_est) * sin(abs(yaw_est) / 2.0f);
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
		qw = imu_msg.orientation.w;
		qx = imu_msg.orientation.x;
		qy = imu_msg.orientation.y;
		qz = imu_msg.orientation.z;

		//SERIAL_PORT.print(F("{\"qw\":"));
		//SERIAL_PORT.print(qw, 3);
		//SERIAL_PORT.print(F(", \"qx\":"));
		//SERIAL_PORT.print(qx, 3);
		//SERIAL_PORT.print(F(", \"qy\":"));
		//SERIAL_PORT.print(qy, 3);
		//SERIAL_PORT.print(F(", \"qz\":"));
		//SERIAL_PORT.print(qz, 3);
		//SERIAL_PORT.println(F("}"));

		// linear_velocity_est * dt と q0,q1,q2,q3 で、ロボットの z軸の移動距離を計算
		sensors.imu_.QuaternionToEulerAngles(qw, qx, qy, qz,roll, pitch, yaw);
		dz = linear_velocity_est * dt * pitch;

		dzi[0] =(int32_t)roundf(dz*1000.0);
		//dz = (double)dzl / 1000.0;

		dzi_sum=0;
		for (int i=0; i<2; i++){
			dzi_sum += dzi[i];
		}
		dz = (double)(dzi_sum/2) / 1000.0;
		for (int i=2-1; i>0; i--){
			dzi[i] = dzi[i-1];
		}

		#endif

		// feed odom message
		//odom.header.stamp = nh.now();
		updateOdometry();
		//odom.header.frame_id = "odom";
		//odom.child_frame_id = "base_footprint";
		odom.pose.pose.position.x += dx;
		odom.pose.pose.position.y += dy;
		odom.pose.pose.position.z = 0.0;
		//odom.pose.pose.position.z += dz;
		odom.pose.pose.orientation.w = qw;
		odom.pose.pose.orientation.x = qx;
		odom.pose.pose.orientation.y = qy;
		odom.pose.pose.orientation.z = qz;
		// Velocity expressed in base_link frame
		odom.twist.twist.linear.x = linear_velocity_est;
		odom.twist.twist.linear.y = 0.0f;
		odom.twist.twist.angular.z = angular_velocity_est;

		//SERIAL_PORT.print(F("{\"x\":"));
		//SERIAL_PORT.print(odom.pose.pose.position.x, 3);
		//SERIAL_PORT.print(F(", \"y\":"));
		//SERIAL_PORT.print(odom.pose.pose.position.y, 3);
		//SERIAL_PORT.print(F(", \"z\":"));
		//SERIAL_PORT.print(odom.pose.pose.position.z, 3);
		//SERIAL_PORT.println(F("}"));

		// error occured -> [ERROR] [1616575654.217167]: Message from device dropped: message larger than buffer.  by nishi
		odom.header.stamp = nh.now();
    	odom_pub.publish(&odom);

		// add by nishi for TF  2021.4.26
 		// odometry tf
  		updateTF(odom_tf);
  		odom_tf.header.stamp = nh.now();
		//odom_tf.header.stamp = odom.header.stamp;

		// ratbmap-nishi_stereo_outdoor.launch と TF がバッテイングする? 2021.9.16
  		tf_broadcaster.sendTransform(odom_tf);
		
		//delay(1);

		// add by nishi for joint states pub. but not yet correct work
		int32_t left_tick=motor_tic.left;
		int32_t right_tick=motor_tic.right;
		//motor_tic.left=0;
		//motor_tic.right=0;
		updateMotorInfo(left_tick,right_tick);
		// joint states
		updateJointStates();
		joint_states.header.stamp = nh.now();
		//joint_states.header.stamp = odom.header.stamp;

		joint_states_pub.publish(&joint_states);

		//Serial.print("motor_tic.left=");
		//Serial.print(motor_tic.left, DEC);
		//Serial.println("");
		//Serial.print("motor_tic.right=");
		//Serial.print(motor_tic.right, DEC);
		//Serial.println("");

		tTime[2] = t + frequency_odometry_hz;	// set 1-cycle-time [ms] to odom Timer.


		t = millis();
    }

	// Update the IMU unit.	add by nishi 2021.7.5
	//if(imu_exec == false){
	//	sensors.updateIMU();
	//	imu_exec=true;
	//}

	// IMU Publish.	add by nishi 2021.7.5
	//if(_frequency_imu.delay(millis())) {
	if (t >= tTime[3]){
	    //publishImuMsg();
		#ifdef USE_MAG
	    publishMagMsg();
		#endif
	    tTime[3] = t + (1000 / FREQUENCY_IMU_PUBLISH_HZ);
		//t = millis();		// comment in by nishi 2022.3.18
	}

	//if(_frequency_ver_info.delay(millis())) {
	//	publishVersionInfoMsg();
	//}

	// update subscribers values
	if (t >= tTime[5]){
		nh.spinOnce();
		// test by nishi
		//uint8_t data[] = "hello4\r\n"; 
		//nh.getHardware()->write(data,8);
	    tTime[5] = t + 2;	// wait 2[ms]
	}

	#if defined(CAMERA_SYNC) or defined(CAMERA_SYNC_EX)
	// got camere info topic. 
	if (camera_info_f == true){
		// 自分(odom publish timestamp) の 時刻は、
		double_t odom_time = odom.header.stamp.toSec();
		// 自分 と camera info(publish timestamp) のズレは?
		double_t pub_off = odom_time - camera_sync_time;
		// 自分 が、遅れています。
		// 時間のズレは、1 cycle 以内です?
		if(pub_off > 0.0 && pub_off <= ((double_t)frequency_odometry_hz / 1000.0)){
			// 自分 は、 camera info より 10[ms] より進んでいます。
			if(pub_off > 0.010){
				// 3[ms] 早くします。
				//tTime[2] = t + frequency_odometry_hz - 3;
				tTime[2] -= 3;
			}
			// 自分 は、camera info より 3[ms] より進んでいます。
			else if(pub_off > 0.003){
				// 1[ms] 早くします。
				//tTime[2] = t + frequency_odometry_hz - 1;
				tTime[2] -= 1;
			}
		}
		// 自分 は、進んでいます。
		// 時間のズレは、1 cycle 以内です?
		else if(pub_off < 0.0 && pub_off >= ((double_t)frequency_odometry_hz / -1000.0)){
			// 自分 は、camera info より 10[ms] より遅いです。
			if(pub_off < -0.010){
				// 3[ms] 遅くします。
				//tTime[2] = t + frequency_odometry_hz + 3;
				tTime[2] += 3;
			}
			// 自分(odom )は、camera info より 3[ms] より遅いです。
			else if(pub_off <  -0.003){
				// 1[ms] 遅くします。
				//tTime[2] = t + frequency_odometry_hz + 1;
				tTime[2] += 1;
			}
		}
		camera_info_f=false;
	}
	#endif
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg) {
	linear_velocity_ref  = cmd_vel_msg.linear.x;
	angular_velocity_ref = cmd_vel_msg.angular.z;
	#if defined(BOARD_F407VG)
	digitalWrite(LED_ORANGE, HIGH-digitalRead(LED_ORANGE));   // blink the led
	#elif defined(BOARD_BLACKPILL) or defined(ESP32)
	digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
	#endif
}

#if defined(CAMERA_SYNC)
void cameraSyncCallback(const sensor_msgs::CameraInfo& camera_info_msg){
	if((camera_info_msg.header.seq % 5) == 0){
		camera_info_f = true;
		//digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
		camera_sync_time = camera_info_msg.header.stamp.toSec();
		if(prev_ok==true){
			// compute camera info publish rate.
			double_t off_time = camera_sync_time - prev_cinfo_stamp;
			if(off_time > 0.0 && camera_info_msg.header.seq > prev_cinfo_sec){
				uint32_t cnt = camera_info_msg.header.seq - prev_cinfo_sec;
				double_t rate = (double_t)cnt / off_time;	// camera info publish rate [Hz]
				if(rate >= 3.0 && rate <= 60.0){
					frequency_odometry_hz_ave += (uint32_t)(1000.0f / rate);
					frequency_odometry_hz_ave /= 2; 
					frequency_odometry_hz = frequency_odometry_hz_ave;	// 1 cycle time [ms]
				}
			}
		}
		prev_cinfo_stamp = camera_sync_time;
		prev_cinfo_sec = camera_info_msg.header.seq;
		prev_ok=true;
	}
}
#elif defined(CAMERA_SYNC_EX)
void cameraSyncCallback(const sensor_msgs::Temperature& temp_msg){
	camera_info_f = true;
	//digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue

	camera_sync_time = temp_msg.header.stamp.toSec();
	camera_cap_hz = temp_msg.temperature;	// get camera caption rate [Hz]
	if(camera_cap_hz > 0.0){
		frequency_odometry_hz = (uint32_t)(1000.0f / camera_cap_hz);	// add by nishi 2022.3.6
	}
}
#endif

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

	debug_left.z = i_epsilon * RATE_CONTROLLER_KI;

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
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

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
}
/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
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
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
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
      sensors.initIMU();
      initOdom();

	  dzi[0]=0;
	  dzi[1]=0;
	  dzi[2]=0;

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
      nh.getParam("~tf_prefix", &get_tf_prefix);

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
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;

	  #if defined(CAMERA_SYNC) or defined(CAMERA_SYNC_EX)
	  camera_info_f = false;
	  #endif
	  #if defined(CAMERA_SYNC)
	  prev_ok=false;
	  #endif
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
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

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

}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;	// odom or odom_fox
  odom_tf.child_frame_id = odom.child_frame_id;	// base_footprint
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
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;

  motor_tic.left=0;		// add by nishi
  motor_tic.right=0;	// add by nishi

  yaw_est=0.;			// add by nishi

}
/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  //joint_states.effort_length   = WHEEL_NUM;
  // update by nishi 2021.5.10
  joint_states.effort_length   = 0;

}

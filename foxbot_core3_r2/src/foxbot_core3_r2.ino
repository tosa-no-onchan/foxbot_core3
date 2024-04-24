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

//#define USE_TRACE
// platformio.ini
// -D USE_TRACE

#if defined(BOARD_ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <analogWrite.h>
#if defined(USE_TRACE)
	#define LED_BUILTIN 4
	HardwareSerial mySerial2(2);	// TX:17  RX:16
#else
	#define LED_BUILTIN 4		// 17 -> Serial2 
#endif
#endif

#if defined(ESP32)
	#include <WiFi.h>
	//const char SSID[] = "WiFi ID";
	//const char PASSWORD[] = "WiFi passsword";
	//IPAddress server(192,168,1,170);
	char server[] = "192.168.1.170";
	uint16_t serverPort = 11411;
	//WiFiClient client;
#endif

//#define ROS2_FOXY
#define ROS2_GALACTIC

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

//void error_loop(){
void error_loop(rcl_ret_t rc){
  int cnt=1;
  while(1){
    #if defined(USE_TRACE)
	  if(cnt>=0){
        mySerial2.print("error_loop() rc:");
        mySerial2.println(rc);
	    cnt--;
	  }
	#endif

    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void error_loop2(String proc,rcl_ret_t rc){
  int cnt=1;
  while(1){
    #if defined(USE_TRACE)
	  if(cnt>0){
        mySerial2.print("error_loop() proc:"+proc+" rc:");
        mySerial2.println(rc);
	    cnt--;
	  }
	  //break;
	#endif

    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void error_trace(String proc,rcl_ret_t rc){
    #if defined(USE_TRACE)
        mySerial2.print("error_trace() proc:"+proc+" rc:");
        mySerial2.println(rc);
	#endif
}

void warn_trace(String proc,rcl_ret_t rc){
    #if defined(USE_TRACE)
        //mySerial2.print("warn "+proc+" rc:");
        mySerial2.print(proc+" rc:");
        mySerial2.println(rc);
	#endif
	// stop moter drive
	linear_velocity_ref=0.0;
	angular_velocity_ref=0.0;
	if(state == AGENT_CONNECTED){
		// AGENT_CONNECT_WATCH
		state =  AGENT_CONNECT_WATCH;
		agent_connect_check_cnt=5;
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

	#if defined(BOARD_ESP32)
		// Builtin LED
		pinMode(LED_PIN, OUTPUT);
		digitalWrite(LED_PIN, HIGH);  // light ON
		delay(500);
		digitalWrite(LED_PIN, LOW);  // light OFF
		delay(500);
		digitalWrite(LED_PIN, HIGH);  // light ON
		delay(500);
		digitalWrite(LED_PIN, LOW);  // light OFF
		delay(500);
	#endif


	#if defined(ESP32)
		Serial.begin(115200);
		delay(10);
		Serial.println("start prog.");

		//set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "192.168.1.57", 8888);
		set_microros_wifi_transports(SSID, PASSWORD, server, serverPort);
	#endif

	//Serial.begin(115200);
	//Serial.begin(1000000);
	//1000000

	//#define ESP32X
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

	#if defined(USE_TRACE)
		mySerial2.begin(115200);
		//mySerial2.begin(9600);		// Jetson Nano 2G /dev/ttyTHS2 ?
		while (!mySerial2) {
			; // wait for serial port to connect. Needed for native USB port only
		}
		mySerial2.println("Start foxbot_core3_r2");
		delay(10);
	#endif

	// start foxbot_core
	//analogWriteResolution(12);
	// changed by nishi 2024.4.24
	analog_write::analogWriteResolution(12);
	#if defined(ESP32)
	    //Serial.println("analogWriteResolution(12) ok");
	#endif

	// set Moter control ports
	initMoter();

	#if defined(BOARD_ESP32)
		// add by nishi for ESP32 only
		//analogWriteChannel(enMotorRight);
		// changed by nishi 2024.4.24
		analog_write::analogWriteChannel(enMotorRight);
		//analogWriteChannel(enMotorLeft);
		// changed by nishi 2024.4.24
		analog_write::analogWriteChannel(enMotorLeft);
	#endif

	// https://micro.ros.org/docs/api/rmw/
	// how to change Serial Speed
	// micro_ros_arduino/src/default_transport.cpp
	//  arduino_trans_open() の中で Srerial.begin(speed) をいじる。
    set_microros_transports();

	delay(2000);

	//#define TEST_2
	// IMU の単体テスト用
	#if defined(TEST_2)
		Serial.begin(115200);
		Serial.println("start IMU test");
		// test IMU start
		for(int i=0;i<4;i++){
			if (0==sensors.initIMU()){
			//if (0==IMU.begin(206)){
				sen_init=true;
				break;
			}
			delay(100);        // delay mill sec 1000 [ms]
		}

		Serial.print("sen_init=");
		Serial.print(sen_init);
		Serial.print("\n");
		delay(1000);        // delay mill sec 1000 [ms]

		while(1){
			//IMU.update();
			sensors.updateIMU();

			Serial.print("W:");
			Serial.print(sensors.imu_.quat[0]);  // W
			Serial.print(" ,X:");
			Serial.print(sensors.imu_.quat[1]);  // X
			Serial.print(" ,Y:");
			Serial.print(sensors.imu_.quat[2]);  // Y
			Serial.print(" ,Z:");
			Serial.print(sensors.imu_.quat[3]);  // Z
			Serial.print("\n");
			//#if defined(IMU_SENSER6)
				delay(2);        // 
			//#else
			//	//delay(30);        // delay mill sec [ms] 33 [hz] -> 
			//	delay(29);        // delay mill sec [ms] 34.48 [hz] -> 
			//	//delay(28);        // delay mill sec [ms] 38 [hz] -> 26 より、取れる。
			//	//delay(26);        // delay mill sec [ms] 38 [hz] -> 途中で、取れなくなる。
			//#endif
		}
	#endif

	//------------------
	// alloc TF Message
	//------------------
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

	#if defined(ROS2_GALACTIC)
		//----------------------------
		// for galactic set up domain id
		//----------------------------
		allocator = rcl_get_default_allocator();
		//rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
		init_options = rcl_get_zero_initialized_init_options();
		RCSOFTCHECK_PROC("setup() #1 : rcl_init_options_init()",
			rcl_init_options_init(&init_options, allocator));

		RCSOFTCHECK_PROC("setup() #2 : rcl_init_options_set_domain_id()",
			rcl_init_options_set_domain_id(&init_options, 30));		// ドメインIDの設定
	#endif

	// Setting for IMU add by nishi 2021.7.5
    #if defined(USE_TRACE)
      mySerial2.println("sensors.init(): Start!!");
	#endif
	if(sensors.init()==0x00){
	  #if defined(USE_TRACE)
		mySerial2.println("sensors.init(): OK");
	  #endif
	}
	else{
	  #if defined(USE_TRACE)
		mySerial2.println("sensors.init(): NG");
	  #endif
	}
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
	trace_prv = AGENT_INIT;	// add by nishi 2023.2.25

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
	  #if defined(USE_TRACE)
		if(trace_prv != WAITING_AGENT){
		  mySerial2.println("WAITING_AGENT");
		  trace_prv = WAITING_AGENT;
		}
	  #endif
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
	  #if defined(USE_TRACE)
		trace_prv = AGENT_AVAILABLE;
	    mySerial2.println("AGENT_AVAILABLE");
	    mySerial2.println("call create_entities()");
	  #endif

      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

      if (state == WAITING_AGENT) {
		#if defined(USE_TRACE)
		  mySerial2.println("error create_entities()");
		#endif
        destroy_entities();

		//delay(1000);

      }
	  else{
		#if defined(USE_TRACE)
		  mySerial2.println("AGENT_CONNECTED");
		#endif
		set_my_time();	// NTP 時刻合わせ

		// set_my_time() wait time
		tTime[7] = foxbot3::micros_() + (5 * 60 * 1000000UL );	// 5[minute]
		tTime[8] = foxbot3::micros_() + (1 * 1000000UL );	// 1[sec]  add by nishi 2023.2.26

		// 初期化は、1回のみにします。 by nishi 2023.2.28
		if(init_cnt==0){
			updateVariable(true);
			updateTFPrefix(true);
			init_cnt++;
		}
	  }

      break;
    case AGENT_CONNECTED:{
		//EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
		// 1度 NG になったら、何回か様子をみる。
		//if(RMW_RET_OK != rmw_uros_ping_agent(100, 1)){
		if(RMW_RET_OK != rmw_uros_ping_agent(100, 3)){	// changed by nishi 2022.12.5
			//state = AGENT_DISCONNECTED;
			state =  AGENT_CONNECT_WATCH;
			agent_connect_check_cnt=5;
		}
		if (state == AGENT_CONNECTED) {
			#if defined(USE_PC_BEAT)
				foxbot3::usec_t t0 = foxbot3::micros_();
				// check pc_beat 
				if(use_beat == true && t0 >= tTime[8]){
					// beat lost
					if(beat_no == beat_no_prev){
						// stop moter drive
						linear_velocity_ref=0.0;
						angular_velocity_ref=0.0;
						beat_ok=false;
						if(beat_ok_prev != beat_ok){
							#if defined(USE_TRACE)
								mySerial2.println("pc_beat stop!!");
							#endif
						}
					}
					// beat active
					else{
						beat_ok=true;
						if(beat_ok_prev != beat_ok){
							#if defined(USE_TRACE)
								mySerial2.println("pc_beat ok!!");
							#endif
						}
					}
					beat_no_prev = beat_no;
					beat_ok_prev = beat_ok;

					tTime[8] = t0 + (1* 1000000UL );	// 1[sec]
				}
			#endif
			// 従来の loop() 処理
			loop_main();
			// 受信 処理
			//rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			//RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));

			// syscronize NTP
			foxbot3::usec_t t = foxbot3::micros_();
			// set_my_time() wait time over
			if(t >= tTime[7]){
				set_my_time(50);
				tTime[7] = t + (5 * 60 * 1000000UL );	// 5[minute]
			}
		}
	  }
      break;

	case AGENT_CONNECT_WATCH:
	  // stop motor
	  linear_velocity_ref=0.0;
	  angular_velocity_ref=0.0;
	  #if defined(USE_TRACE)
		if(trace_prv != AGENT_CONNECT_WATCH){
		  mySerial2.println("AGENT_CONNECT_WATCH start");
		  trace_prv = AGENT_CONNECT_WATCH;
		  mySerial2.println("stop moter drive");
		}
	  #endif
	  //if(RMW_RET_OK != rmw_uros_ping_agent(100, 3)){	// changed by nishi 2022.12.5
	  if(RMW_RET_OK != rmw_uros_ping_agent(100, 5)){	// changed by nishi 2023.2.26
		//state = AGENT_DISCONNECTED;
		if(agent_connect_check_cnt<=0){
			state = AGENT_DISCONNECTED;
		}
		agent_connect_check_cnt--;
	  }
	  else{
		state = AGENT_CONNECTED;
		trace_prv=AGENT_CONNECTED;
		#if defined(USE_TRACE)
		  mySerial2.println("AGENT_CONNECT_WATCH OK");
		#endif

	  }
	  break;

    case AGENT_DISCONNECTED:
	  #if defined(USE_TRACE)
		mySerial2.println("AGENT_DISCONNECTED");
	  #endif
	  // リセットは、止めます。 by nishi 2023.2.28
	  //updateVariable(false);
	  //updateTFPrefix(false);
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
	// add by nishi 2024.2.11
	#if defined(USE_AGENT_LED)
		if (state == AGENT_CONNECTED) {
			digitalWrite(LED_BUILTIN, LOW);	// light OFF
		} 
		else {
			digitalWrite(LED_BUILTIN, HIGH);	// light ON
		}
	#endif
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

		#if !defined(ODOM_USE_IMU)
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

			// 上で、q[2] をいじったら必要です。Noramalizeする。by nishi 2022.2.20
			#if defined(USE_Q2_adjust)
				q[0]=1.0 - ((q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
				if(q[0] >= 0.0) q[0] = sqrt(q[0]);
				else q[0] = sqrt(q[0] * -1.0) * -1.0;
			#endif


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
		RCSOFTCHECK_PROC("loop_main() :#1 odom_publish",rcl_publish(&odom_publisher, &odom, NULL));

		// add by nishi 2022.9.9
		// use_tf_static==true : publist tf odom -> base_footprint 
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
			// publish エラーの時は、
			RCSOFTCHECK_PROC("loop_main() :#2 tf_publish",rcl_publish(&tf_publisher, tf_message, NULL));
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
		//RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
		RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));	// update by nishi 2023.1.19

	    //tTime[5] = t + 2000;	// wait 2[ms]
	    tTime[5] = t + (1000000UL / FREQUENCY_ROSPINONCE_HZ);	// wait 20[ms]
	}

}


//twist message cb
void commandVelocityCallback(const void *cmd_vel_msg){
	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)cmd_vel_msg;
	// if velocity in x direction is 0 turn off LED, if 1 turn on LED
	//digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
	if(state == AGENT_CONNECTED && beat_ok==true){
		linear_velocity_ref  = ((const geometry_msgs__msg__Twist *)cmd_vel_msg)->linear.x;
		angular_velocity_ref = ((const geometry_msgs__msg__Twist *)cmd_vel_msg)->angular.z;
	}
}

// pc_beat cb add by nishi 2023.3.2
void pc_beatCallback(const void *pc_beat_msg){
	const std_msgs__msg__UInt32 * msg = (const std_msgs__msg__UInt32 *)pc_beat_msg;
	beat_no = msg->data;
}

bool create_entities()
{
	rcl_ret_t rc;
	bool ok=true;


	#if defined(USE_ORG1)
		allocator = rcl_get_default_allocator();
		//----------------------
		// create init_options
		//----------------------
		RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
		//----------------------
		// create node
		//----------------------
		RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
	#elif defined(ROS2_FOXY)
		//
		// reffer from  https://qiita.com/ousagi_sama/items/a814b3db925b7ce2aeea
		//----------------------------
		// for foxy
		//----------------------------
		allocator = rcl_get_default_allocator();
		//----------------------
		// create init_options
		//----------------------
		RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

		//----------------------
		// create node
		//----------------------
		// ノードの生成 (foxy版)
		rcl_node_options_t node_ops = rcl_node_get_default_options();
		node_ops.domain_id = (size_t)(30);		// ドメインIDの設定
		//rclc_node_init_with_options(&node, "node_name", "namespace", &support, &node_ops);
		rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops);

	#else
		//----------------------------
		// for galactic
		//----------------------------
		// 此処は、1 回のみ実行します。
		// 再接続の時は、ここは、実行しては、いけない。
		// setup() の中で、行います。
		//
		//	allocator = rcl_get_default_allocator();
		//
		//	init_options = rcl_get_zero_initialized_init_options();
		//	RCSOFTCHECK_PROC("create_entities() #1 : rcl_init_options_init()",
		//		rcl_init_options_init(&init_options, allocator));
		//
		//	RCSOFTCHECK_PROC("create_entities() #2 : rcl_init_options_set_domain_id()",
		//		rcl_init_options_set_domain_id(&init_options, 30));		// ドメインIDの設定
		//
		//----------------------
		// create init_options
		//----------------------
		// 上の処理を毎回行うと、再接続の時、ここで、エラー rc=1
		RCCHECK_PROC("create_entities() #3 : rclc_support_init_with_options()",
			rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
			// 			
			// RCL_RET_INVALID_ARGUMENT: 11
			// RCL_RET_ERROR: 1

		//----------------------
		// create node
		//----------------------
		//rclc_node_init_default(&node, "node_name", "namespace", &support);
		RCCHECK_PROC("create_entities() #4 : rclc_node_init_default()",
			rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

	#endif

	//----------------------
	// create publisher
	//----------------------

	// create publisher for "odom_fox"
	RCCHECK_PROC("create_entities() #5 : init odom_publisher",
		rclc_publisher_init_default(
			&odom_publisher, 
			&node,
			//odom_type_support,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), 
			odom_topic_name));

	// create publisher for "tf"
	RCCHECK_PROC("create_entities() #6 : init tf_publisher",
		rclc_publisher_init_default(
			&tf_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
			tf_topic_name));

	// create publisher for "IMU"
	RCCHECK_PROC("create_entities() #7 : init imu_publisher",
		rclc_publisher_init_best_effort(
			&imu_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			imu_topic_name));

	#if defined(USE_FOX_BEAT)
		// create publisher for "fox_beat"
		RCCHECK_PROC("create_entities() #8 : init fox_beat_publisher",
			rclc_publisher_init_default(
				&fox_beat_publisher, 
				&node,
				//odom_type_support,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), 
				fox_beat_topic_name));
	#endif


	// create subscriber for Twist "cmd_vel"
	RCCHECK_PROC("create_entities() #9 : init cmd_vel_subscriber",
		rclc_subscription_init_default(
			&cmd_vel_subscriber,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"cmd_vel"));

	#if defined(USE_PC_BEAT)
		// create subscriber for UInt32 "pc_beat" add by nishi 2023.3.2
		RCCHECK_PROC("create_entities() #10 : init pc_beat_subscriber",
			rclc_subscription_init_default(
				&pc_beat_subscriber,
				&node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
				"pc_beat"));
	#endif


	//----------------------
	// create timer
	//----------------------

	const unsigned int timer_timeout = 1000;
	RCCHECK_PROC("create_entities() #11 : init timer",
		rclc_timer_init_default(
			&timer,
			&support,
			RCL_MS_TO_NS(timer_timeout),
			timer_callback));

	//----------------------
	// create executor
	//----------------------
	//  for publisher
	#if !defined(USE_PC_BEAT)
		RCCHECK_PROC("create_entities() #12 : init executor",
			rclc_executor_init(&executor, &support.context, 1, &allocator));
	#else
		RCCHECK_PROC("create_entities() #12 : init executor",
			rclc_executor_init(&executor, &support.context, 2, &allocator));
	#endif

	//  for cmd_vel subscriber
	RCCHECK_PROC("create_entities() #13 : executor_add_subscription executor",
		rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg0, &commandVelocityCallback, ON_NEW_DATA));

	#if defined(USE_PC_BEAT)
		//  for pc_beat subscriber add by nishi 2023.3.2 by nishi
		RCCHECK_PROC("create_entities() #14 : executor_add_subscription executor",
			rclc_executor_add_subscription(&executor, &pc_beat_subscriber, &pc_beat_msg0, &pc_beatCallback, ON_NEW_DATA));
	#endif

	return ok;
}

void destroy_entities()
{
  rcl_ret_t rc;
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rc=rcl_publisher_fini(&odom_publisher, &node);
  rc=rcl_publisher_fini(&tf_publisher, &node);
  rc=rcl_publisher_fini(&imu_publisher, &node);

  #if defined(USE_FOX_BEAT)
    rc=rcl_publisher_fini(&fox_beat_publisher, &node);
  #endif

  rc=rcl_subscription_fini(&cmd_vel_subscriber, &node);

  #if defined(USE_PC_BEAT)
    rc=rcl_subscription_fini(&pc_beat_subscriber, &node);	// add by nishi 2023.3.2
  #endif

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
			#if defined(IMU_SENSER6)
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
			#else
				//tTime[6] = t + 1000000UL / 60;   //    
				//tTime[6] = t + 1000000UL / 50;   //    
				tTime[6] = t + 1000000UL / 38;   // 38[Hz]
				//tTime[6] = t + 1000000UL / 34;	// 34.48[Hz] 2024.4.23 setteing original
			#endif
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
		//analogWrite(enMotor, pwm_ref,PWM_MAX);
		// chnaged by nishi 2024.4.24
		analog_write::analogWrite(enMotor, pwm_ref,PWM_MAX);
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
* https://github.com/micro-ROS/micro_ros_arduino/issues/881
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();		// Now dummy
  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

  //imu_pub.publish(&imu_msg);
  RCSOFTCHECK_PROC("publishImuMsg(): #1 imu_publish",rcl_publish(&imu_publisher, &imu_msg, NULL));

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
	  sen_init=false;

	  #if defined(USE_TRACE)
    	mySerial2.println("sensors.initIMU(): Start!!");
	  #endif

	  for(int i=0;i<4;i++){
      	if (0==sensors.initIMU()){
	      sen_init=true;
          break;
		}
	    //delay(1000);        // delay mill sec 1000 [ms]
	    delay(500);        // delay mill sec 1000 [ms]
	  }

	  #if defined(USE_TRACE)
	  	if(sen_init==true){
	    	mySerial2.println("sensors.initIMU(): OK");
		}
		else{
	    	mySerial2.println("sensors.initIMU(): NG");
		}
	  #endif

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
      //nh.getParam("~use_tf_static", &yes_no);
      if (use_tf_static == false){
		sprintf(log_msg, "Setup use_tf_static [false]");
		//nh.loginfo(log_msg); 
	  }
	  else{
		sprintf(log_msg, "Setup use_tf_static [true]");
		//nh.loginfo(log_msg); 
	  }

      //nh.getParam("~use_imu_pub", &yes_no);
      if (use_imu_pub == true){
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

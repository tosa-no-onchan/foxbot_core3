# foxbot_core3 for ros2  
![foxbot_core3](https://github.com/tosa-no-onchan/foxbot_core3/blob/main/image/DSC03485-base2-b.jpg)  
自作 Turtlebot SOC プログラムです。  
ESP32 で、モーター駆動、IMU のデータ取り込みと、Host SBC への転送をおこないます。  
Host SBC との通信は、Esp32 Serial - Orange Pi 5 Serial で、micro-ROS for Arduino を利用します。  

foxbot_core3 (ros2 galactic) 版を、ros2 humble 向けに改造しました。  
対応: ros2 humble and jazzy  

詳しい内容は、  
[ROS2 で、ESP32(Arduino) を使う。](http://www.netosa.com/blog/2022/10/ros2-esp32arduino.html)  
[Turtlebot3 自作の最近のブログ記事](http://www.netosa.com/blog/cat2/ros/robot-car/)  
[ROS2 Turtlebot3 自作の最近のブログ記事](http://www.netosa.com/blog/cat2/ros2/ros2-turtlebot3/)  

Host SBC Control プログラムは、  
[turtlebot3_navi_my for Ros2 humble r6](https://github.com/tosa-no-onchan/turtlebot3_navi_my)  

Launch ファイルは、  
[tosa-no-onchan/rtabmap_ros_my](https://github.com/tosa-no-onchan/rtabmap_ros_my)  
  
Static Map Navigation  
[foxbot_nav2_oak-d_depth_gps.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/foxbot_nav2_oak-d_depth_gps.launch.py)  
  
Active Slam  
[rtabmap_oak-d_rgb_depth_gps.launch.py](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/rtabmap_oak-d_rgb_depth_gps.launch.py)  


  
#### [開発環境]  
    
    PC  
      Ubuntu Mate 22.04 and 24.04  
      ESP32  Arduino  
      Vscode and Platformio IDE  
      ros2 humble and jazzy  
      micro-ROS for Arduino  

#### [動作環境]  
    
    Remote PC  
      Ubuntu Mate 22.04 and 24.04  
      ros2 humble,jazzy  
    Host SBC  
      Orange Pi 5 Armibian Jummy  
      Ros2 humble and jazzy  
      micro-ROS Agent  
    SOC  
      ESP32  
      micro-ROS  
      IMU: ICM-20948
    

#### 1. download this proj.  
    
    $ git clone https://github.com/tosa-no-onchan/foxbot_core3.git  
    $ cd foxbot_core3  

#### 2. copy foxbot_core3_r2 to vscode Platformio IDE Project  
    
    $ copy -ar foxbot_core3_r2 ~/Documents/PlatformIO/Projects  


#### 3. copy Arduino/lib-foxbot_core3_r2 to ~/Arduino
    
    $ cd Arduino  
    $ cp -ar lib-foxbot_core3_r2 ~/Arduino  
  
#### 4. copy foxbot_tool to ~/colcon_ws/src
    
    $ cp -ar foxbot_tool ~/colcon_ws/src  
    $ cd ~/colcon_ws  
    $ colcon build --symlink-install --packages-select foxbot_tool  
  

#### 5. download Arduino other Library 
    
    $ cd ~/Arduino/lib-foxbot_core3_r2  
    
    5.1 down load SparkFun_ICM-20948_ArduinoLibrary  
    $ git clone SparkFun_ICM-20948_ArduinoLibrary to here  
    
    5.2 down load micro-ROS for Arduino  
    $ git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git  

    5.2 down load Arduino ESP32_AnalogWrite  
    find website Arduino ESP32_AnalogWrite and down load and unzip to ~/Arduino/lib-foxbot_core3_r2  
    insert namespace{} top and bottom of source  
    edit ESP32_AnalogWrite/src/analogWrite.cpp and src/analogWrite.h  
    namespace analog_write {  
    ...  
    }  

    5.3 down load BNO086 SparkFun ArduinoLibrary  
    $ git clone https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library.git  to here 

#### 6. Import source code to Vscode Platformio IDE project  
start vscode  
    
    PlatformIO::Home  
      Import Arduino Project or Open project  
      Espressif ESP32 Dev Module  
      select ~/Documents/PlatformIO/Projects/foxbot_core3_r2  

#### 7. Change something 
Edit ~/Arduino/lib-foxbot_core3_r2/micro_ros_arduino/src/default_transport.cpp  
    
    ....   
    bool arduino_transport_open(struct uxrCustomTransport * transport)  
    {  
      //Serial.begin(115200);  
      // changed by nishi  
      Serial.begin(1000000);    // 1M  
      return true;  
    }  
    
#### 8. Chose foxbot_core3_r2 pubish topics    
Edit foxbot_core3_r2_config.h  
    
    // 2) use odom topic name which '/odom' or '/odom_fox'  add by nishi 2023.4.4
    #define USE_ODOM_FOX

    #if defined(USE_ODOM_FOX)
        const char * odom_topic_name = "odom_fox";
    #else
        const char * odom_topic_name = "odom";
    #endif

    // add by nishi 2022.9.9    
    // use_tf_static==true : publist tf odom -> base_footprint   
    bool use_tf_static=true;    
    // use_imu_pub==true : publist 'imu'   
    bool use_imu_pub=false;    

#### 9. How to Run    
On SBC  
      
      $ sudo chmod 777 /dev/ttyTHS1  
      $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTHS1 -b 1000000  

On Remote PC  
      
      $ sudo ufw disable  
      $ ros2 topic list  
      #$ ros2 run foxbot_tool heart_beat  
      $ ros2 run turtlebot3_teleop teleop_keyboard

C++ programable robot control  
[turtlebot3_navi_my](https://github.com/tosa-no-onchan/turtlebot3_navi_my)        
   
#### 10. Update    
2023.2.28  
      
      1) ESP32 Serial2 is used for Trace.  
        TX:17  RX:16  
      2) LED is 14 
      3) galactic 版になってから、通信断時の再接続が、途中でエラーになっていた点の修正。  
      4) 再接続した際に、従来は、 IMUのリセット、odom、TF の位置、向き情報の初期化をしていましたが、それを止めにしました。  
        IMUのリセット、odom、TF の位置、向き情報の初期化 は、ESP32 が起動した時のみとしました。  
      5) 通信断になっている間は、モータ駆動を即止める。  
      これで、ロボットの位置、向き情報が継続されるので、通信エラーが有っても無視できます。  
      注) ROS のほうでも、この問題が出ているものと思います。いずれ、バックポートしなければいかんぞね。  

2023.3.16  
      
      1) heart beat を追加。  
        /pc_beat が、Publish されている時のみ、/cmd_vel を取り込みます。  
        
2024.4.24  
      
      1) ROS2 humble 版を公開しました。  
      heart beat は、使いません。  

2025.1.26  
      
      1) update_motor() と loop_main() 間での、sensors.copyIMU() 実行時の  
      排他制御 を、portMUX_TYPE mutex に変更しました。  
      こうちらのほうが、latency が無くて、快適です。   

``````
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
portENTER_CRITICAL(&mutex);
sensors.copyIMU();
portEXIT_CRITICAL(&mutex);
``````

2025.3.17  

      1) ICM-20948 DMP6 and DMP9 のキャリブレーション時に、起動時の初期誤差を  
      補正する処理を追加しました。  

2025.3.24  

      1) ICM-20948 DMP6 and DMP9 を止めて、ACC、Gyro、 Magnet + MadgwickAHRS を使うことにした。  
    
2025.5.25  

      1) ICM-20948 ACC、Gyro + MadgwickAHRS 6軸 Fusion を使うことにした。  
    


これに関する記事。  
[ROS2 自作 Turtlebot3 による 草刈りロボット開発。#11 ロボットの走行方向がずれる。](https://www.netosa.com/blog/2025/03/ros2-turtlebot3-11.html)  

2025.6.25  

      1) BNO086 DMP 9軸 Fusion を使うことにした。  
      $ git clone https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library.git to ~/Arduino/lib-foxbot_core3_r2  
      Edit ~/Arduino/lib-foxbot_core3_r2/IMU/IMU.h  
``````
      //#include "ICM20948.h"  
      #include "BNO086.h"  

``````


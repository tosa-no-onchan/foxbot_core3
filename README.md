# foxbot_core3 for ros2
![foxbot_core3](https://github.com/tosa-no-onchan/foxbot_core3/blob/main/image/DSC03485-base2-b.jpg)  
foxbot_core3 (ros2 galactic) 版を、ros2 humble 向けに改造しました。 

詳しい内容は、  
[ROS2 で、ESP32(Arduino) を使う。](http://www.netosa.com/blog/2022/10/ros2-esp32arduino.html)  
[Turtlebot3 自作の最近のブログ記事](http://www.netosa.com/blog/cat2/ros/robot-car/)  
[ROS2 Turtlebot3 自作の最近のブログ記事](http://www.netosa.com/blog/cat2/ros2/ros2-turtlebot3/)  
  
#### [開発環境]  
    
    PC  
      Ubuntu Mate 22.04  
      ESP32  Arduino  
      Vscode and Platformio IDE  
      ros2 humble  
      micro-ROS for Arduino  

#### [動作環境]  
    
    Remote PC  
      Ubuntu Mate 22.04  
      ros2 humble  
    SBC  
      Orange Pi 5 Armibian Jummy  
      Ros2 humble  
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
        
      

### foxbot_core3 for ros2
foxbot_core3 (ros noetic) 版を、ros2 foxy 向けに改造しました。 

詳しい内容は、[こちら。](http://www.netosa.com/blog/2022/10/ros2-esp32arduino.html)  
  
#### [開発環境]  
    
    PC  
      Ubuntu Mate 20.04  
      ESP32  Arduino  
      Vscode and Platformio IDE  
      ros2 galactic  
      micro-ROS for Arduino  

#### [動作環境]  
    
    Remote PC  
      Ubuntu Mate 20.04  
      ros2 galactic  
    SBC  
      Jetson Nano 2G  
      JetPack 4.x and Ubuntu 20.04  
      Ros2 galactic  
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
  
  

#### 4. download Arduino other Library 
    
    $ cd ~/Arduino/lib-foxbot_core3_r2  
    
    4.1 down load SparkFun_ICM-20948_ArduinoLibrary  
    $ git clone SparkFun_ICM-20948_ArduinoLibrary to here  
    
    4.2 down load micro-ROS for Arduino  
    $ git clone -b foxy https://github.com/micro-ROS/micro_ros_arduino.git  

#### 5. Import source code to Vscode Platformio IDE project  
start vscode  
    
    PlatformIO::Home  
      Import Arduino Project or Open project  
      Espressif ESP32 Dev Module  
      select ~/Documents/PlatformIO/Projects/foxbot_core3_r2  

#### 6. Change something 
Edit ~/Arduino/lib-foxbot_core3_r2/micro_ros_arduino/src/default_transport.cpp  
    
    ....   
    bool arduino_transport_open(struct uxrCustomTransport * transport)  
    {  
      //Serial.begin(115200);  
      // changed by nishi  
      Serial.begin(1000000);    // 1M  
      return true;  
    }  
    
#### 7. Chose foxbot_core3_r2 pubish topics    
Edit foxbot_core3_r2_config.h  
    
    // add by nishi 2022.9.9    
    // use_tf_static==true : publist tf odom -> base_footprint   
    bool use_tf_static=true;    
    // use_imu_pub==true : publist 'imu'   
    bool use_imu_pub=false;    

#### 8. How to Run    
On SBC  
      
      $ sudo chmod 777 /dev/ttyTHS1  
      $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTHS1 -b 1000000  

On Remote PC  
      
      $ sudo ufw disable  
      $ ros2 topic list  
      $ ros2 run turtlebot3_teleop teleop_keyboard
   
#### 9. Change LOG    
2023.2.28  
      
      ESP32 Serial2 is used for Trace.  
      TX:17  RX:16  
      LED is 14  

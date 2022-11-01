# foxbot_core3 for ros2
foxbot_core3 (ros noetic) 版を、ros2 foxy 向けに改造しました。 
   
1. download  
  $ git clone https://github.com/tosa-no-onchan/foxbot_core3.git  
  
2. copy foxbot_core3_r2 to VS Platfome IDE Project  
  $ copy -ar foxbot_core3_r2 ~/Documents/PlatformIO/Projects  
  
3. copy Arduino/lib-foxbot_core3_r2 to your-home-dir  
  $ cd Arduino  
  $ cp -ar lib-foxbot_core3_r2 ~/Arduino  
  
  $ cd ~/Arduino/lib-foxbot_core3_r2  
  3.1 down load SparkFun_ICM-20948_ArduinoLibrary  
  $ cd ~/Arduino/lib-foxbot_core3_r2  
  $ git clone SparkFun_ICM-20948_ArduinoLibrary to here  
  
  3.2 down load micro-ROS for Arduino  
  $ git clone -b foxy https://github.com/micro-ROS/micro_ros_arduino.git  




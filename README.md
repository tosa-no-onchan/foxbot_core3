# foxbot_core3
[foxbot_core](https://github.com/MattMgn/foxbot_core) を、 ESP32 WiFi or Serial 用に改造しました。 
 
詳しい内容は、[こちら。](http://www.netosa.com/blog/2021/05/turtlebot3-4.html)  
[一連の記事](http://www.netosa.com/blog/cat2/ros/robot-car/)  
  
### 関連サイト。Thanks!!  
1. [Build Your Own Turtlebot Robot!](https://www.instructables.com/Build-Your-Own-Turtblebot-Robot/) 
2. [Build your own TurtleBot 3 backbone](https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone)
3. [MattMgn/foxbot_core](https://github.com/MattMgn/foxbot_core)
4. [MattMgn/arduino-libraries](https://github.com/MattMgn/arduino-libraries)
5. [ESP32_AnalogWrite](https://github.com/ERROPiX/ESP32_AnalogWrite)
6. [turtlebot3_core.ino](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_waffle/turtlebot3_core/turtlebot3_core.ino)
  
### Turtlebot3 として、SLAMを試す。  
カメラも、liDAR もついていませんが、SLAM の Rviz での確認ができます。  

    1) $ roscore  
    2) $ roslaunch turtlebot3_bringup turtlebot3_robot.launch  --screen  
      注1) turtlebot3_core.launch の記述のデバイスを変更します  
      /dev/ttyACM0 を tcp に変更します。  
      注2) Foxbot_coreと通信できると、トピックの一覧が表示されます。  
    3) SLAMの実行  
    $ export TURTLEBOT3_MODEL=waffle  
    $ roslaunch turtlebot3_slam turtlebot3_slam.launch  
    4) Rviz の起動。  
    $ export TURTLEBOT3_MODEL=waffle  
    $ rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz  
      注) Rviz の設定  
    Robot Model と Odometry を有効にします。  
    5) キーボード操作のプログラムを起動。  
    $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  

注1) liDAR、カメラが付いていないので、残念ながら、Slam の出力である、map は、パブリッシュされません。  

### IMU サポートと SBC(Jetson Nano 2G) との Serial 通信をサポート。  
Turtlebot3 としての機能を組み込みました。
SBC に 自作 Stereo Camera や Depth Camera を取り付ければ、Turtlebot3 and rtabmap_ros を体験できます。  
    
    1) IMU:ICM-20948 DMP 6軸Fusion に対応しました。  
    2) IMU 6軸Fusion から、/odom_fox と tf-base_footprint の Orientationの向きに 値を設定しています。  
       ロボットの位置は、モータの駆動距離から、向きは IMU DMP 6軸 Fusion から計算して、Publish しています。  
       注) robot_localization/ekf_localization_node は、必要ありません。  
    3) Camera Info に同期して、/odom_fox , tf-base_footprint をパブリッシュします。  
    
### SBC(Jetson Nano 2G) rtabmap_ros with 自作 Stereo Camera    
SBC(Jestosn Nano 2G + ESP32 Seial) で、Turtlebot3 with ratbmap_ros Stereo camera(Single USB Stereo Camera) が実行できます。    
[rtabmap_ros_my/launch/rtabmap-nishi_stereo_outdoor2_bz3_navi.launch](https://github.com/tosa-no-onchan/rtabmap_ros_my/blob/main/launch/rtabmap-nishi_stereo_outdoor2_bz3_navi.launch) を、参考にして下さい。    
    
難点.    
    
    1) ESP32 - Jetson Nano 2G の Serial通信に難あり。    
       時々ロボットの /odom_fox tf-base_footprintが通信断になります。        
    2) ICM20948 DMP の初期化にかなりの頻度で失敗します。    
       LED が点灯すると、初期化を失敗しているので、    
       $ roslaunch turtlebot3_bringup turtlebot3_robot.launch  --screen    
       を再度実行して下さい。    
    
Update.    
    
    2022.3.21    
        /odom_fox tf-base_footprint の publish 時間のタイマーを milli sec から micro sec に変えて、    
        精度を上げました。    
    
    
 



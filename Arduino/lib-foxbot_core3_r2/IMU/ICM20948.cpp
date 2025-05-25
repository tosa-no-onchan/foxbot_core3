/*
* ICM20948.cpp
*/
#include <Arduino.h>
#include <SPI.h>

#include "ICM20948.h"


//#define SPI_CS_PIN          BDPIN_SPI_CS_IMU
                            
#define ICM20948_ADDRESS    0xEA
#define MPU_CALI_COUNT      512


//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

//#define DEBUG_M

float cICM20948::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

cICM20948::cICM20948()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;

  // ICM20948
  // ACCELEROMETER 
  // 1.Full-Scale Range
  //  ACCEL_FS=0  -> ±2 [G]
  //  ACCEL_FS=1  -> ±4 [G]
  //  ACCEL_FS=2  -> ±8 [G]
  //  ACCEL_FS=3  -> ±16 [G]
  // 2.ADC Word Length Output in two’s complement format 16 Bits 1
  // 3.Sensitivity Scale Factor
  //  ACCEL_FS=0 -> 16,384 [LSB/g]
  //  ACCEL_FS=1 -> 8,192 [LSB/g]
  //  ACCEL_FS=2 -> 4,096 [LSB/g]
  //  ACCEL_FS=3 -> 2,048 [LSB/g]
  //
  // GYROSCOPE
  // 1.Full-Scale Range 
  //GYRO_FS_SEL=0 ±250 dps 1
  //GYRO_FS_SEL=1 ±500 dps 1
  //GYRO_FS_SEL=2 ±1000 dps 1
  //GYRO_FS_SEL=3 ±2000 dps 1
  // 2.Gyroscope ADC Word Length 16 bits 1
  // 3.Sensitivity Scale Factor
  //GYRO_FS_SEL=0 131 LSB/(dps) 1
  //GYRO_FS_SEL=1 65.5 LSB/(dps) 1
  //GYRO_FS_SEL=2 32.8 LSB/(dps) 1
  //GYRO_FS_SEL=3 16.4 LSB/(dps) 1
  //
  // Magnet
  // 1.Full-Scale Range ±4900 μT 1
  // 2.Output Resolution 16 bits 1
  // 3.Sensitivity Scale Factor 0.15 μT / LSB 1

  // ACC Z 軸の Zero 調整
  // Z 軸の停止時の中心が、 0 になるように調整します。
  zero_off = ACC_ZERO_OFF;

  #if defined(USE_DMP_NISHI) && defined(USE_AGM_NISHI)
    begin_update_rate = 30;
  #elif defined(USE_DMP_NISHI) && !defined(IMU_SENSER6)
    begin_update_rate = 30;
    //begin_update_rate = 50;
  #elif defined(USE_DMP_NISHI) && defined(IMU_SENSER6)
    begin_update_rate = 30;
  #elif defined(USE_MAG)
    //begin_update_rate = 90;
    //begin_update_rate = 10;
    //begin_update_rate = 100;
    #if defined(USE_DUAL_RATE)
      begin_update_rate = 500;
      //begin_update_rate = 100;
    #else
      begin_update_rate = 100;
    #endif
    mag_update_rate = 100;
    //mag_update_rate = 10;
    mag_update_rate_us = 1000000/mag_update_rate;

  #else
    //begin_update_rate = 300;
    //begin_update_rate = 400;
    begin_update_rate = 500;
    //begin_update_rate = 600;  // 実測 565[Hz]
  #endif
  magADC[0]=magADC[1]=magADC[2]=0;
  magADC_BD[0]=magADC_BD[1]=magADC_BD[2]= 0.0;
}

bool cICM20948::begin()
{
  int cnt;
  uint8_t data;

  #if defined(USE_SPI)
    // test by nishi MOSI(23) Pull UP 2023.12.13
    //pinMode(23,INPUT_PULLUP);
    #if defined(BOARD_ESP32)
      SPI_PORT.begin(18,19,23,5);    // SCLK,MISO,MOSI,CS
    #elif defined(BOARD_PICO32)
      SPI_PORT.begin(18,36,26,19);    // SCLK,MISO,MOSI,CS
    #endif
    // test by nishi MOSI(23) Pull UP 2023.12.13
    //pinMode(23,INPUT_PULLUP);
  #else
    // Wire.begin(sda,scl)
    //WIRE_PORT.begin();
    WIRE_PORT.begin(21,22);
    WIRE_PORT.setClock(400000);
  #endif


  bool initialized = false;
  bConnected=false;
  
  //cnt = 2;
  cnt = 4;  // changed by nishi 2023.3.4
  while (!initialized)
  {

    #if defined(USE_SPI)
      // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/issues/87
      // myICM1.begin(CS_PIN_1, SPI_PORT, 7000000);
      myICM.begin(CS_PIN, SPI_PORT);
      // test by nishi 2025.4.1
      //myICM.begin(CS_PIN, SPI_PORT,2000000);

    #else
      myICM.begin(WIRE_PORT, AD0_VAL);
    #endif

    // add by nisshi 2024.2.17
    //myICM.swReset();
    //delay(250);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
      //SERIAL_PORT.println("please Ent!");
      //WAITFORINPUT()
      cnt--;
      if(cnt <=0){
        return false;
      }
    }
    else
    {
      initialized = true;
    }
  }

  #if defined(DO_SWRESET)
    // 注) myICM.swReset() したら、 DNP-6 and acc,gyro,mag が取れなくなるので、使わない!! by nishi 2025.4.17
    // add by nishi 2024.2.19
    // Here we are doing a SW reset to make sure the device starts in a known state
    // リセットしたら、再度、myICM.startupMagnetometer() が必要です。
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("Software Reset returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }
    delay(250);

    // add by nishi 2025.4.17
    myICM.startupMagnetometer();
    delay(250);
  #endif

  // add by nishi 2025.4.4
  //myICM.resetDMP();
  //delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // Disable I2C interface, use SPI
  //spiWriteByte(ICM20948_REG_USER_CTRL, ICM20948_BIT_I2C_IF_DIS);

  uint8_t whoami;
  cnt = 4;
  while(cnt >=0){
    //whoami = spiReadByte(ICM20948_REG_WHO_AM_I);
    whoami = myICM.getWhoAmI();
    // whoami = 0xEA  -> ICM20948
    if(whoami == 0xEA)
      break;
    delay(500);
    cnt--;
  }
  // whoami = 0xEA  -> ICM20948
  //if(whoami == ICM20948_DEVICE_ID)
  if(whoami == 0xEA)
  {
    bConnected = init();
    if(bConnected==true){
      gyro_init();
      acc_init();
      mag_init();
      #if defined(USE_DMP_NISHI)
        dmp_init();
      #endif
    }
    //MPU_SPI.setClockDivider( SPI_CLOCK_DIV4 ); // 6.5MHz
  }
  // add by nishi
  else{
    SERIAL_PORT.print("cICM20948::begin(): #1 whoami=");
    SERIAL_PORT.println(whoami, HEX);
    //while(1){
    //  delay(100);
    //}
  }

  return bConnected;
}

bool cICM20948::init( void ){
  uint8_t state;
  uint8_t data;
  uint8_t response[3] = {0, 0, 0};

  bool rc=true;

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  // リセットしたら、再度、myICM.startupMagnetometer() が必要です。
  // 下記は、USE_AGM_NISHI の時は、使わない事。
  #if !defined(USE_AGM_NISHI)
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("Software Reset returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }
    delay(250);
    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);
  #endif

  #if !defined(USE_DMP_NISHI) && !defined(USE_AGM_NISHI)
    // Gyro and Acc を使う
    #if defined(USE_ACC_NISHI) || defined(USE_GRYO_NISHI) || defined(USE_MAG)

      // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

      // Set Gyro and Accelerometer to a particular sample mode
      // options: ICM_20948_Sample_Mode_Continuous
      //          ICM_20948_Sample_Mode_Cycled
      myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        SERIAL_PORT.print(F("setSampleMode returned: "));
        SERIAL_PORT.println(myICM.statusString());
        rc = false; // add by nishi 2022.4.23
      }

      // Set full scale ranges for both acc and gyr
      ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

      //myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16
      #if defined(USE_ACC_2G)
        myFSS.a = gpm2;  // gpm2
      #elif defined(USE_ACC_4G)
        myFSS.a = gpm4;  // gpm4
      #else
        myFSS.a = gpm8;  // gpm8
      #endif

      #if defined(USE_GYRO_250)
        myFSS.g = dps250;
      #elif defined(USE_GYRO_500)
        myFSS.g = dps500;
      #elif defined(USE_GYRO_1000)
        myFSS.g = dps1000;
      #else
        myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                        // dps250
                        // dps500
                        // dps1000
                        // dps2000
      #endif

      myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        SERIAL_PORT.print(F("setFullScale returned: "));
        SERIAL_PORT.println(myICM.statusString());
        rc = false; // add by nishi 2022.4.23
      }


      // Set up Digital Low-Pass Filter configuration
      ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
      //myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)   original
      //myDLPcfg.a = acc_d246bw_n265bw;     // acc_d246bw_n265bw  -> 7. NG    - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
      //myDLPcfg.a = acc_d111bw4_n136bw;   // acc_d111bw4_n136bw -> 6. OK!
      //myDLPcfg.a = acc_d50bw4_n68bw8; // acc_d50bw4_n68bw8 -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
      //myDLPcfg.a = acc_d23bw9_n34bw4;  // acc_d23bw9_n34bw4 -> 4.  so so
      //myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Good! ただし #3のばあい
      //myDLPcfg.a =  acc_d5bw7_n8bw3;  // acc_d5bw7_n8bw3   - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz  -> 2 NG
                                      // acc_d473bw_n499bw   original  1. soso

      #if defined(USE_IMU_NO1)
        myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Good! ただし #3のばあい and use for #2
      #elif defined(USE_IMU_NO2)
        myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Madgwick は、こちら!!
      #else
        // IMU NO3
        //myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)   original
        //myDLPcfg.a = acc_d246bw_n265bw;     // acc_d246bw_n265bw  -> 7. NG   - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
        myDLPcfg.a = acc_d111bw4_n136bw;   // acc_d111bw4_n136bw -> 6. OK!
        //myDLPcfg.a = acc_d50bw4_n68bw8; // acc_d50bw4_n68bw8 -> 5. Good! ただし #2 のばあい。
        //myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Good! ただし #3のばあい and use for #2
        //myDLPcfg.a =  acc_d5bw7_n8bw3;  // acc_d5bw7_n8bw3   - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz  -> 2 NG
      #endif

      //myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)  original
                                        // gyr_d196bw6_n229bw8
      //myDLPcfg.g =  gyr_d151bw8_n187bw6;  // gyr_d151bw8_n187bw6  -> 7. NG
      //myDLPcfg.g = gyr_d119bw5_n154bw3;   // gyr_d119bw5_n154bw3  -> 6. OK 
      //myDLPcfg.g =  gyr_d51bw2_n73bw3;  // gyr_d51bw2_n73bw3  -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
      //myDLPcfg.g =  gyr_d23bw9_n35bw9;    // gyr_d23bw9_n35bw9   -> 4.  so so
      //myDLPcfg.g = gyr_d11bw6_n17bw8;   // gyr_d11bw6_n17bw8  -> 3. Good!  ただし #3のばあい
      //myDLPcfg.g = gyr_d5bw7_n8bw9;     // gyr_d5bw7_n8bw9  -> 2. NG
                                        // gyr_d361bw4_n376bw5  original  1. so so
      #if defined(USE_IMU_NO1)
        myDLPcfg.g = gyr_d11bw6_n17bw8;   // gyr_d11bw6_n17bw8  -> 3. Good! Madgwick は、もうちょい。 Mahony は、こちら。まあまあ!!
      #elif defined(USE_IMU_NO2)
        // IMU NO2 は、OK
        myDLPcfg.g = gyr_d5bw7_n8bw9;     // gyr_d5bw7_n8bw9  -> 2. #2 Madgwick は、こちら!!
      #else
        //IMU NO3　は、もうすこし!!
        //myDLPcfg.g =  gyr_d151bw8_n187bw6;  // gyr_d151bw8_n187bw6  -> 7.  NG
        //myDLPcfg.g = gyr_d119bw5_n154bw3;   // gyr_d119bw5_n154bw3  -> 6. NG
        //myDLPcfg.g =  gyr_d51bw2_n73bw3;  // gyr_d51bw2_n73bw3  -> 5. NG
        myDLPcfg.g =  gyr_d23bw9_n35bw9;    // gyr_d23bw9_n35bw9   -> 4.  so so  Mahonyは、もうちょい
        //myDLPcfg.g = gyr_d11bw6_n17bw8;   // gyr_d11bw6_n17bw8  -> 3. Good! Madgwick は、もうちょい。 Mahony は、こちら。まあまあ!!
        //myDLPcfg.g = gyr_d5bw7_n8bw9;     // gyr_d5bw7_n8bw9  -> 2. NG.
      #endif

      myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
      //myICM.setDLPFcfg(ICM_20948_Internal_Acc , myDLPcfg);
      if (myICM.status != ICM_20948_Stat_Ok){
        SERIAL_PORT.print(F("setDLPcfg returned: "));
        SERIAL_PORT.println(myICM.statusString());
        rc = false; // add by nishi 2022.4.23
      }

      // Choose whether or not to use DLPF
      // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
      //ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
      ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
      //ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
      ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
      SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
      SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
      SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
      SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));


      // Choose whether or not to start the magnetometer
      #if defined(USE_MAG)
        myICM.startupMagnetometer();
        if (myICM.status != ICM_20948_Stat_Ok){
          SERIAL_PORT.print(F("startupMagnetometer returned: "));
          SERIAL_PORT.println(myICM.statusString());
          rc = false; // add by nishi 2022.4.23
        }
      #endif

      // test by nishi
      myICM.setBank(2); // myICM.startupMagnetometer(); の後は、Bank が変わる。
      uint8_t reg_my;
      ICM_20948_Status_e rc_my = myICM.read(AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg_my, sizeof(reg_my));
      SERIAL_PORT.print("reg_my=");
      SERIAL_PORT.println(reg_my,HEX);

      SERIAL_PORT.println();
      SERIAL_PORT.println(F("Configuration complete!"));

      // DEBUG by nishi
      #if defined(DEBUG_NISHI_8)
        // bank0 USER_CTRL 0x03
        myICM.setBank(0); // myICM.startupMagnetometer(); の後は、Bank が変わる。
        rc_my = myICM.read(0x03, (uint8_t *)&data, sizeof(data));
        SERIAL_PORT.print("USER_CTRL=");
        SERIAL_PORT.println(data,HEX);

        while(1){
          delay(100);
        }
      #endif
      //SERIAL_PORT.println("Please CR!");
      //WAITFORINPUT();
    #endif
  #else
    /*
    * add 9 axis fusion Quarternion with DMP3 start
    * 注) ICM220948C.h の #define ICM_20948_USE_DMP を有効にしないといけない。
    * 注2) 上記を、有効にすると、Magnet が取れなくなる。
    */

    bool success = true; // Use success to show if the DMP configuration was successful

    //#define QUAT_ANIMATION

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    #if defined(USE_AGM_NISHI)
      // Enable the DMP Game Rotation Vector sensor (Quat6)
      //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
      // Enable additional sensors / features
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
      //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);   // original  2021.11.11

    // Enable the DMP orientation sensor
    #elif !defined(IMU_SENSER6)
      // 2025.3.14 以前は、こちらを使用
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);   // original  2021.11.11
      // 2025.3.14 から、こちらに変えてみます。
      //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);   // test 2 -> OK 2024.2.19
    #else
      //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GRAVITY) == ICM_20948_Stat_Ok);   // test 3
      success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);   // test 4
    #endif

    // Enable any additional sensors / features
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    // (55/34) -1 = 0.6  -> int
    // (55/9.16666....) -1 = 5
    #if defined(USE_AGM_NISHI)
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 1) == ICM_20948_Stat_Ok);        // Set to 1Hz
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 1) == ICM_20948_Stat_Ok);         // Set to 1Hz
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 10) == ICM_20948_Stat_Ok); // Set to 5Hz
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 1) == ICM_20948_Stat_Ok); // Set to 27Hz

    #elif !defined(IMU_SENSER6)
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum -> Max speed is Bad
      // test by nishi 2025.1.15
      // (55/9.16) -1 = 5 -> これ迄の設定値
      // (55/18.3) -1 = 2 ->  OK
      // (55/27.5) -1 = 1 ->  OK  ちょっと不安定か?
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok); // test by nishi 2025.1.23
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 2) == ICM_20948_Stat_Ok); // test by nishi 2025.1.23
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    #else
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) == ICM_20948_Stat_Ok); // Set to the 27.5
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
      //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    #endif

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
      #if !defined(QUAT_ANIMATION)
        SERIAL_PORT.println(F("DMP enabled!"));
      #endif
    }
    else
    {
      rc=false;
      SERIAL_PORT.println(F("Enable DMP failed!"));
      SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      //while (1)
      //  ; // Do nothing more
    }
  #endif

  // end
  return rc;
}


bool cICM20948::update( void ){
  bool rc=false;
  mag_data_comming=false;
  #if !defined(USE_DMP_NISHI) && !defined(USE_AGM_NISHI)
    if( bConnected == true ){
      if (myICM.dataReady()){
        // AGMT の中で、新しいデータがない時は、同じデータがくるみたい。
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'

        #if defined(USE_MAG) && defined(USE_DUAL_RATE)
          mag_cur_process_time  = micros();
          mag_process_time = mag_cur_process_time-mag_prev_process_time;
          if(mag_process_time >= mag_update_rate_us){
            mag_data_comming=true;
            mag_prev_process_time = mag_cur_process_time;
          }
        #elif defined(USE_MAG)
          mag_data_comming=true;
        #endif
        rc=true;
      }
      //delayMicroseconds(10); 
    }
  #else
    if( bConnected == true ){
      rc=true;
    }
  #endif
  
  return rc;
}

void cICM20948::gyro_init( void ){
	uint8_t i;
	for( i=0; i<3; i++ ){
    gyroADC[i]  = 0;
		gyroZero[i] = 0;
		gyroRAW[i]  = 0;
	}
  calibratingG_f = 0;
  calibratingG = 0; 
	//calibratingG = MPU_CALI_COUNT;
}

void cICM20948::gyro_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

  if( bConnected == true ){
    // scale data Gyr (DPS)
    gyroADC_BD[0] = myICM.gyrX();
    gyroADC_BD[1] = myICM.gyrY();
    gyroADC_BD[2] = myICM.gyrZ();

    //#define USE_GYRO_DEBUG_X1
    #if defined(USE_GYRO_DEBUG_X1)
      SERIAL_PORT.print(F("gyroSCL: "));
      SERIAL_PORT.print(gyroADC_BD[0], 3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(gyroADC_BD[1], 3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(gyroADC_BD[2], 3);
    #endif


    // raw data
  	gyroRAW[0] = x = myICM.agmt.gyr.axes.x;
  	gyroRAW[1] = y = myICM.agmt.gyr.axes.y;
  	gyroRAW[2] = z = myICM.agmt.gyr.axes.z;

    //#define USE_GYRO_DEBUG_X2
    #if defined(USE_GYRO_DEBUG_X2)
      // 一応、同じになる。
      //float xf,yf,zf;
      //xf = (float)x * gRes;
      //yf = (float)y * gRes;
      //zf = (float)z * gRes;
      SERIAL_PORT.print(F("gyroRAW:"));
      //SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(x);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(y);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(z);
      SERIAL_PORT.println("");
    #endif

  	GYRO_ORIENTATION( x, y,z );
  }
  gyro_common();
}

void cICM20948::gyro_cali_start(){
	//calibratingG = MPU_CALI_COUNT;
  calibratingG_f = 0;
  calibratingG = 0; 
}

void cICM20948::gyro_common(){
	static int32_t g[3];

  //static int cnt_s=0;

  if(calibratingG_f == 0){
    calibratingG++;
    if(calibratingG >= MPU_CALI_COUNT_GYRO_PRE){
      if(calibratingG == MPU_CALI_COUNT_GYRO_PRE){
        g[0]=0;
        g[1]=0;
        g[2]=0;
        //cnt_s=0;
      }
      //cnt_s++;
			g[0] += gyroADC[0];             // Sum up 512 readings
			g[1] += gyroADC[1];             // Sum up 512 readings
			g[2] += gyroADC[2];             // Sum up 512 readings

      if(calibratingG >= (MPU_CALI_COUNT_GYRO_PRE+MPU_CALI_COUNT_GYRO)-1){
        gyroZero[0] = g[0] / MPU_CALI_COUNT_GYRO;
        gyroZero[1] = g[1] / MPU_CALI_COUNT_GYRO;
        gyroZero[2] = g[2] / MPU_CALI_COUNT_GYRO;

        //#define USE_GYRO_DEBUG_X3
        #if defined(USE_GYRO_DEBUG_X3)
          // 一応、同じになる。
          //float xf,yf,zf;
          //xf = (float)x * gRes;
          //yf = (float)y * gRes;
          //zf = (float)z * gRes;
          SERIAL_PORT.print(F("calibratingG:"));
          SERIAL_PORT.print(calibratingG);
          SERIAL_PORT.print(F(" cnt_s:"));
          SERIAL_PORT.print(cnt_s);
          SERIAL_PORT.println("");
          while(1)
            delay(100);
        #endif

        //gyroZeroSum=gyroZero[0]+gyroZero[1]+gyroZero[2];
        calibratingG_f=1;
        calibratingG=0;
        // test by nishi 2025.3.25
        //gyroZero[0]=gyroZero[1]=gyroZero[2]=0;
      }
    }
  }
  if(calibratingG_f != 0){
    // これがないと、ドリフトが大きいみたい。
    gyroADC[0] -= gyroZero[0];
    gyroADC[1] -= gyroZero[1];
    gyroADC[2] -= gyroZero[2];

    //#define USE_GYRO_DEBUG_X4
    #if defined(USE_GYRO_DEBUG_X4)
      SERIAL_PORT.print(F("gyroADC:"));
      //SERIAL_PORT.print(gyroADC[0]);
      //SERIAL_PORT.print(F(" ,"));
      //SERIAL_PORT.print(gyroADC[1]);
      //SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(gyroADC[2]);
      SERIAL_PORT.println("");
    #endif

    // GYRO 0 noise cut off
    // MadgwickAHRS.cpp and FusionAhrs.c では、gyro値 = 0.0 だとアクセスエラーと
    // 判定するみたいだ。 by nishi 2022.5.12
    // ノイズを削ってみる。
    //for (int axis = 0; axis < 3; axis++){
    //  if (abs(gyroADC[axis]) <= GYRO_NOISE_CUT_OFF){
    //    gyroADC[axis] = 0;
    //  }
    //}
    if (abs(gyroADC[2]) <= GYRO_NOISE_CUT_OFF){
      gyroADC[2] = 0;
      //gyroADC[2] /= 4;
    }

    //#define USE_GYRO_DEBUG_X5
    #if defined(USE_GYRO_DEBUG_X5)
      SERIAL_PORT.print(F("gyroADC:"));
      //SERIAL_PORT.print(gyroADC[0]);
      //SERIAL_PORT.print(F(" ,"));
      //SERIAL_PORT.print(gyroADC[1]);
      //SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(gyroADC[2]);
      SERIAL_PORT.println("");
    #endif
  }
}

void cICM20948::acc_init( void ){
  uint8_t i;
  for( i=0; i<3; i++ ){
    accADC[i]   = 0;
		accZero[i]  = 0;
    accRAW[i]   = 0;
  }

  calibratingA_f = 0;
  calibratingA = 0;    // add by nishi 2022.1.23
  accIMZero=0.0;
}

void cICM20948::acc_get_adc( void ){
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
  //uint8_t rawADC[6];

  // scaled data Acc [mg]
  accADC_BD[0] = myICM.accX();
  accADC_BD[1] = myICM.accY();
  accADC_BD[2] = myICM.accZ();

  //#define USE_ACC_DEBUG_X1
  #if defined(USE_ACC_DEBUG_X1)
    //SERIAL_PORT.print(F("accSCL:"));
    SERIAL_PORT.print(F("Raw:"));
    SERIAL_PORT.print(accADC_BD[0], 3);
    SERIAL_PORT.print(F(","));
    SERIAL_PORT.print(accADC_BD[1], 3);
    SERIAL_PORT.print(F(","));
    SERIAL_PORT.print(accADC_BD[2], 3);
  #endif

  accRAW[0] = x = myICM.agmt.acc.axes.x;
  accRAW[1] = y = myICM.agmt.acc.axes.y;
  accRAW[2] = z = myICM.agmt.acc.axes.z;

  //#define USE_ACC_DEBUG_X2
  #if defined(USE_ACC_DEBUG_X2)
    SERIAL_PORT.print(F("Raw:"));
    SERIAL_PORT.print(accRAW[0]);
    SERIAL_PORT.print(F(","));
    SERIAL_PORT.print(accRAW[1]);
    SERIAL_PORT.print(F(","));
    SERIAL_PORT.print(accRAW[2]);
  #endif

  // レゾルーションを掛けた値?  agmt.fss.a を元に、G に変換した値
  //switch (agmt.fss.a){
  //case 0: -> (((float)axis_val) / 16.384);
  //case 1: -> (((float)axis_val) / 8.192);
  //case 2: -> (((float)axis_val) / 4.096);
  //case 3: -> (((float)axis_val) / 2.048);

  ACC_ORIENTATION( x,	y, z );
	acc_common();
}

/*
* get Average of Acc value
* from start to 512 
* get Average for each acc x,y,z
* but z is set to zero;
*/
void cICM20948::acc_common(){
	static int32_t a[3];

  if(calibratingA_f == 0){
    calibratingA++;
    if(calibratingA >= MPU_CALI_COUNT_ACC_PRE){
      // Z 軸のノイズを取り除く
      //if(abs(accADC[2] - ACC_1G) > ACC_ZERO_Z_OVER){
      //  calibratingA--;
      //  return;
      //}
      if(calibratingA == MPU_CALI_COUNT_ACC_PRE){
        a[0]=0;
        a[1]=0;
        a[2]=0;
      }
			a[0] += accADC[0];             // Sum up 512 readings
			a[1] += accADC[1];             // Sum up 512 readings
			a[2] += accADC[2];             // Sum up 512 readings

      if(calibratingA >= (MPU_CALI_COUNT_ACC_PRE+MPU_CALI_COUNT_ACC -1)){
        accZero[0] = a[0] / MPU_CALI_COUNT_ACC;
        accZero[1] = a[1] / MPU_CALI_COUNT_ACC;
        accZero[2] = a[2] / MPU_CALI_COUNT_ACC;
        // test by nishi 2025.3.25
        //accZero[0]=accZero[1]=accZero[2]=0;

        accZeroSum=accZero[0]+accZero[1]+accZero[2];

        // 此処で、acc の内積を出す。
        //accIMZero = sqrt(accZero[0] * accZero[0] + accZero[1] * accZero[1] + accZero[2] * accZero[2]);
        //accZero[YAW] -= ACC_1G;
        //accZero[YAW] = 0;   // 注) これをすると、海抜からの標高値になる。しなければ、起動地点の標高が原点となる。

        calibratingA_f=1;
        calibratingA=0;
      }
    }
  }
  if(calibratingA_f != 0){
    //accADC[0] -=accZero[0];
    //accADC[1] -=accZero[1];
    //accADC[2] -=accZero[2];
  }
}

void cICM20948::mag_init( void )
{
  uint8_t i;
  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
  }
  calibratingM_f = 0;
  calibratingM = 0;
}

/*---------------------------------------------------------------------------
     TITLE   : mag_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cICM20948::mag_get_adc( void )
{
	//int16_t x = 0;
	//int16_t y = 0;
	//int16_t z = 0;

  //uint8_t data[8];

  //magADC[0]=magADC[1]=magADC[2]=0.0f;

  bool rc=false;

  //#define USE_MAG_DEBUG_X1
  #if defined(USE_MAG_DEBUG_X1)
    SERIAL_PORT.println(F("cICM20948::mag_get_adc():#1"));
  #endif

  mx0 =0.0;
  my0 =0.0;
  mz0 =0.0;

  if( bConnected == true && mag_data_comming == true)
  {
    // あたらしいデータがない時は、前の値がくるみたい。

    //if (data[7] & MPU9250_AK8963_OVERFLOW)
  	if (myICM.agmt.magStat2 & 0x80)
    {
      //SERIAL_PORT.println("mag_get_adc() : #3 overflow");
      magRAW[0] = 0;
      magRAW[1] = 0;
      magRAW[2] = 0;
      magADC[0] = 0;
      magADC[1] = 0;
      magADC[2] = 0;
      return false;
    }


    magRAW[0] = myICM.agmt.mag.axes.x;
    magRAW[1] = myICM.agmt.mag.axes.y;
    magRAW[2] = myICM.agmt.mag.axes.z;
    if(magRAW[0]==0 && magRAW[1]==0 && magRAW[2]==0)
      return false;

    // scaled data Mag (uT)  1桁下げる。
    //magADC_BD[0]=myICM.magX() * 0.1;
    //magADC_BD[1]=myICM.magY() * 0.1;
    //magADC_BD[2]=myICM.magZ() * 0.1;

    magADC_BD[0]=myICM.magX();  // +_4900
    magADC_BD[1]=myICM.magY();
    magADC_BD[2]=myICM.magZ();

    //#define USE_MAG_DEBUG_X1
    #if defined(USE_MAG_DEBUG_X1)
      SERIAL_PORT.print(F("MagADC:"));
      SERIAL_PORT.print(int(magADC_BD[0]));
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(int(magADC_BD[1]));
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(int(magADC_BD[2]));
    #endif

    //#define USE_MAG_DEBUG_X2
    #if defined(USE_MAG_DEBUG_X2)
      SERIAL_PORT.print(F("Raw:0,0,0,0,0,0,"));
      SERIAL_PORT.print(magADC_BD[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(magADC_BD[1]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(magADC_BD[2]);
      //SERIAL_PORT.print(F(","));
      //SERIAL_PORT.println(accuracy);
      SERIAL_PORT.println("");
    #endif


    //  AK8963_ASA[3] -> AK8963 キャリブレーションデータだが、AK09916 は、無し。
    //magRAW[0] = ((long)magRAW[0] * AK8963_ASA[0]) >> 8;
    //magRAW[1] = ((long)magRAW[1] * AK8963_ASA[1]) >> 8;
    //magRAW[2] = ((long)magRAW[2] * AK8963_ASA[2]) >> 8;

    //#define USE_MAG_DEBUG_X4
    #if defined(USE_MAG_DEBUG_X4)
      SERIAL_PORT.print(F("MagRaw:"));
      SERIAL_PORT.print(magRAW[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(magRAW[1]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(magRAW[2]);
      //SERIAL_PORT.print(F(","));
      //SERIAL_PORT.print(accuracy);
      SERIAL_PORT.println("");
    #endif

    //#define USE_MAG_DEBUG_X5
    #if defined(USE_MAG_DEBUG_X5)
      SERIAL_PORT.print(F("Raw:0,0,0,0,0,0,"));
      SERIAL_PORT.print(magRAW[0]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(magRAW[1]);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(magRAW[2]);
      //SERIAL_PORT.print(F(","));
      //SERIAL_PORT.println(accuracy);
      SERIAL_PORT.println("");
    #endif

    mag_common();
    rc=true;
	}
  return rc;
}

void cICM20948::mag_common()
{
	static long m[3];  // 4900.0 / 0.15 = 32666.6  , 32,666.6*100 = 32,66,666.6

  if(calibratingM_f == 0){
    calibratingM++;
    if(calibratingM >= MPU_CALI_COUNT_MAG){

      if(calibratingM == MPU_CALI_COUNT_ACC){
        m[0]=0;
        m[1]=0;
        m[2]=0;
      }
			m[0] += magRAW[0];   // Sum up 512 readings
			m[1] += magRAW[1];   // Sum up 512 readings
			m[2] += magRAW[2];   // Sum up 512 readings

      if(calibratingM >= (MPU_CALI_COUNT_MAG*2 - 1)){
        magZero[0] = m[0] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
        magZero[1] = m[1] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
        magZero[2] = m[2] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant

        // test by nishi 2025.3.25
        //mag_offsets[0] = (float)magZero[0] * mRes * -1.0; 
        //mag_offsets[1] = (float)magZero[1] * mRes * -1.0;
        //mag_offsets[2] = (float)magZero[2] * mRes * -1.0;

        //mag_offsets[0] = (float)magZero[0];
        //mag_offsets[1] = (float)magZero[1];
        //mag_offsets[2] = (float)magZero[2];

        calibratingM_f=1;
        calibratingM=0;
      }
    }
  }
  if(calibratingM_f == 1){

    #define USE_MOTION_CALI
    #if defined(USE_MOTION_CALI)
      float x = (float)magRAW[0] - mag_offsets[0];  // raw data
      float y = (float)magRAW[1] - mag_offsets[1];  // raw data
      float z = (float)magRAW[2] - mag_offsets[2];  // raw data
    #else
      // こちらは、Mahony がキャリブレーションが終わるまで、時間がかかる。
      float x = magADC_BD[0] - mag_offsets[0];  // scale data
      float y = magADC_BD[1] - mag_offsets[1];  // scale data
      float z = magADC_BD[2] - mag_offsets[2];  // scale data
    #endif

    //#define USE_MAG_DEBUG_X10
    #if defined(USE_MAG_DEBUG_X10)
      SERIAL_PORT.print(F("xyz:"));
      SERIAL_PORT.print(x);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(y);
      SERIAL_PORT.print(F(","));
      SERIAL_PORT.println(z);
    #endif


    // Apply mag soft iron error compensation
    //mx0 = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    //my0 = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    //mz0 = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

    mx0 = (x * mag_softiron_matrix[0][0])+(y * mag_softiron_matrix[0][1])+(z * mag_softiron_matrix[0][2]);
    my0 = (x * mag_softiron_matrix[1][0]) + (y * mag_softiron_matrix[1][1]) + (z * mag_softiron_matrix[1][2]);
    mz0 = (x * mag_softiron_matrix[2][0]) + (y * mag_softiron_matrix[2][1]) + (z * mag_softiron_matrix[2][2]);

    //magADC_BD[0] = (float)magRAW[0] * mRes;
    //magADC_BD[1] = (float)magRAW[1] * mRes;
    //magADC_BD[2] = (float)magRAW[2] * mRes;

    //mx0 *= mRes;
    //my0 *= mRes;
    //mz0 *= mRes;

    // normalize
    //double magIMZero = sqrt(magADC_BD[0] * magADC_BD[0] + magADC_BD[1] * magADC_BD[1] + magADC_BD[2] * magADC_BD[2]);
    //magADC_BD[0] /= magIMZero;
    //magADC_BD[1] /= magIMZero;
    //magADC_BD[2] /= magIMZero;

    // normalize
    float magIMZero = sqrt(mx0 * mx0 + my0 * my0 + mz0 * mz0);
    mx0 /= magIMZero;
    my0 /= magIMZero;
    mz0 /= magIMZero;
    

    //#define USE_MAG_DEBUG_X11
    #if defined(USE_MAG_DEBUG_X11)
      SERIAL_PORT.print(F("mx0-mz0: "));
      SERIAL_PORT.print(mx0);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(my0);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(mz0);
      // mag_offsets: 90.600 ,48.900 ,55.650
      //SERIAL_PORT.print(F(" ,"));
      //SERIAL_PORT.println(accuracy);
    #endif
    #if defined(USE_MAHONY)
      // Mahony
      // my は、極性を逆にしない。
      my0 = my0 * -1.0;
      // mz は、逆にする。
      mz0 = mz0 * -1.0;
    #elif defined(USE_MADWICK) || defined(USE_MADWICK2)
      // madgwick
      // my は、極性を逆にしない。
      my0 = my0 * -1.0;
      // mz は、逆にする。
      mz0 = mz0 * -1.0;
    #endif
  }

  magADC[0] =  magRAW[0];  //
  magADC[1] =  magRAW[1];  //
  magADC[2] =  magRAW[2];  //

  // Sensitivity Scale Factor = 0.15 add by nishi 2021.11.4
  //  Full Range +_4900 [uT]
  //magADC_BD[0] =  (float)(magRAW[0] - magZero[0]) * MAG_UT_LSB;  // [uT]
  //magADC_BD[1] =  (float)(magRAW[1] - magZero[1]) * MAG_UT_LSB;  // [uT]
  //magADC_BD[2] =  (float)(magRAW[2] - magZero[2]) * MAG_UT_LSB;  // [uT]

  //#define USE_MAG_DEBUG_X12
  #if defined(USE_MAG_DEBUG_X12)
    SERIAL_PORT.print(F("magADC: "));
    SERIAL_PORT.print(magADC[0], 3);
    SERIAL_PORT.print(F(" ,"));
    SERIAL_PORT.print(magADC[1], 3);
    SERIAL_PORT.print(F(" ,"));
    SERIAL_PORT.println(magADC[2], 3);
    //SERIAL_PORT.print(F(" ,"));
    //SERIAL_PORT.println(accuracy);
  #endif

}

#if defined(USE_DMP_NISHI)

void cICM20948::dmp_init( void )
{
  uint8_t i;

  for( i=1; i<4; i++ )
  {
    quat[i]   = 0.0;
		quatZero[i]  = 0.0;
    quatRAW[i]   = 0.0;
  }
  quat[0]   = 1.0;
  // add by nishi 2025.3.17
  quatZero[0]  = 1.0;
  quatRAW[0]   = 1.0;

  calibratingD_f = 0;
  calibratingD = 0;    // add by nishi 2022.1.23

}

bool cICM20948::dmp_cali_get_done()
{
	if( calibratingD_f == 1 ) return true;
	else                    return false;
}


bool cICM20948::dmp_get_adc(){

  //#define TRACE_DMP11
  #if defined(TRACE_DMP11)
    SERIAL_PORT.println("dmp_get_adc() start");
  #endif

  /*
  * add 9 axis fusion Quarternion with DMP3
  */
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.

  bool rc=false;
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  //static int32_t d[4];
  // changed by nishi 2025.3.14
  static double d[4];
  
  static byte f=0;
  static double q[4]={1.0, 0.0, 0.0, 0.0};

  #define QUAT_CALIB

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    #if defined(TRACE_DMP11)
      SERIAL_PORT.println("dmp_get_adc() #3");
    #endif

    #if !defined(IMU_SENSER6)
      // DMP 9 Fusion
      // ここで、良く取れない時がある。!!
      if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
      {

        #if defined(TRACE_DMP1)
          SERIAL_PORT.println("dmp_get_adc() #4");
        #endif

        // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
        // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
        // ドリフトする時は、合計が、 1 になっていないので 正しい bias values で、 quaternion data を補正しないといけない。
        // The quaternion data is scaled by 2^30.

        //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

        // Scale to +/- 1
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30  -- X
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30  -- Y
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30  -- Z
        //double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W    こいつが、バグとの事。

        double q0=1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
        if(q0 >= 0.0) q0 = sqrt(q0);
        else q0 = sqrt(q0 * -1.0) * -1.0;

        // nomarize してみる。
        // https://github.com/rawify/Quaternion.cpp
        //Quaternion x(1, 2, 3, 4);
        //x.normalize();
        //sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    	  #if defined(USE_NORMAR)
          double iLen = 1.0 / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
          q0 *= iLen;
          q1 *= iLen;
          q2 *= iLen;
          q3 *= iLen;
        #endif

        //#define TEST_W0_X
        #if defined(TEST_W0_X)
          //SERIAL_PORT.print(F("#5 Q:"));
          //SERIAL_PORT.print(q0, 3);
          //SERIAL_PORT.print(F(" ,"));
          //SERIAL_PORT.print(q1, 3);
          //SERIAL_PORT.print(F(" ,"));
          //SERIAL_PORT.print(q2, 3);
          //SERIAL_PORT.print(F(" ,"));
          //SERIAL_PORT.print(q3, 3);
          SERIAL_PORT.print(F(" Accuracy:"));
          SERIAL_PORT.println(data.Quat9.Data.Accuracy);
        #endif

        // 1500 -> NG
        // 1200 -> NG
        // 1100 -> NG
        //if(!isnan(q0) && (data.Quat9.Data.Accuracy >= 0 && data.Quat9.Data.Accuracy <= 500)){
        // 2025.4.3 までの設定
        //if(!isnan(q0) && (data.Quat9.Data.Accuracy >= 0 && data.Quat9.Data.Accuracy <= 600)){
        if(!isnan(q0)){

          //#define TRACE_DMP1_X
          #if defined(TRACE_DMP1_X)
            SERIAL_PORT.println("dmp_get_adc() #5 OK");
          #endif
                                              //   x x x x x
          quatRAW[0] = q0;    // changed by nishi 2025.3.14
          //quatRAW[1] = data.Quat9.Data.Q1;    // 1 1 2 2 3 3
          quatRAW[1] = q1;    // changed by nishi 2025.3.14
          //quatRAW[2] = data.Quat9.Data.Q2;    // 2 3 1 3 1 2
          quatRAW[2] = q2;    // changed by nishi 2025.3.14
          //quatRAW[3] = data.Quat9.Data.Q3;    // 3 2 3 1 2 1
          quatRAW[3] = q3;    // changed by nishi 2025.3.14


          //#define TEST_W0_X2
          #if defined(TEST_W0_X2)
            SERIAL_PORT.print(F("#6 Q1:"));
            SERIAL_PORT.print(quatRAW[1]);
            SERIAL_PORT.print(F(" Q2:"));
            SERIAL_PORT.print(quatRAW[2]);
            SERIAL_PORT.print(F(" Q3:"));
            SERIAL_PORT.println(quatRAW[3]);
            SERIAL_PORT.print(F(" Accuracy:"));
            SERIAL_PORT.println(data.Quat9.Data.Accuracy);
          #endif

          // キャリブレーション中です。
          if(calibratingD_f == 0){
            calibratingD++;
            if(calibratingD >= MPU_CALI_COUNT_DMP9){
              if(calibratingD == MPU_CALI_COUNT_DMP9){
                d[1]=0;
                d[2]=0;
                d[3]=0;
              }
              d[1] += quatRAW[1];             // Sum up 512 readings
              d[2] += quatRAW[2];             // Sum up 512 readings
              d[3] += quatRAW[3];             // Sum up 512 readings

              // キャリブレーションの終了回数に到達した。
              if(calibratingD >= MPU_CALI_COUNT_DMP9*2-1){
                // DMP の初期誤差を計算する。
                // Z軸のみ補正します。by nishi 2025.3.17
                //quatZero[1] = d[1] / MPU_CALI_COUNT_DMP9;
                quatZero[1] = 0.0;
                //quatZero[2] = d[2] / MPU_CALI_COUNT_DMP9;
                quatZero[2] = 0.0;
                quatZero[3] = d[3] / MPU_CALI_COUNT_DMP9; 

                calibratingD_f = 1;
                calibratingD = 0;

                //#define TEST_W0_X3
                #if defined(TEST_W0_X3)
                  SERIAL_PORT.print(F("#7 Q1:"));
                  SERIAL_PORT.print(quatRAW[1]);
                  SERIAL_PORT.print(F(" Q2:"));
                  SERIAL_PORT.print(quatRAW[2]);
                  SERIAL_PORT.print(F(" Q3:"));
                  SERIAL_PORT.println(quatRAW[3]);
                  SERIAL_PORT.print(F(" Accuracy:"));
                  SERIAL_PORT.println(data.Quat9.Data.Accuracy);
                  //while(1){
                  //  delay(100);
                  //}
                #endif
              }
            }
            // キャリブレーションが終了した直後です。
            if (calibratingD_f == 1)
            {
              //#define TEST_W0_X4
              #if defined(TEST_W0_X4)
                SERIAL_PORT.print(F("#8 Q1:"));
                SERIAL_PORT.print(quatRAW[1]);
                SERIAL_PORT.print(F(" Q2:"));
                SERIAL_PORT.print(quatRAW[2]);
                SERIAL_PORT.print(F(" Q3:"));
                SERIAL_PORT.println(quatRAW[3]);

                SERIAL_PORT.print(F("QZ1:"));
                SERIAL_PORT.print(quatZero[1]);
                SERIAL_PORT.print(F(" QZ2:"));
                SERIAL_PORT.print(quatZero[2]);
                SERIAL_PORT.print(F(" QZ3:"));
                SERIAL_PORT.println(quatZero[3]);
                //SERIAL_PORT.print(F(" Accuracy:"));
                //SERIAL_PORT.println(data.Quat9.Data.Accuracy);
              #endif

              // DMP の誤差 quat の実数部を求める。
              quatZero[0] =1.0 - ((quatZero[1] * quatZero[1]) + (quatZero[2] * quatZero[2]) + (quatZero[3] * quatZero[3]));
              if(quatZero[0] >= 0.0) quatZero[0] = sqrt(quatZero[0]);
              else quatZero[0] = sqrt(quatZero[0] * -1.0) * -1.0;

              // DMP 誤差 quat の補正(共役)  --> 誤差の分だけ、反対方向に回転させる。
              quatZeroK[1] = quatZero[1] * -1.0;
              quatZeroK[2] = quatZero[2] * -1.0;
              quatZeroK[3] = quatZero[3] * -1.0;
              //quatZeroK[0] =1.0 - ((quatZeroK[1] * quatZeroK[1]) + (quatZeroK[2] * quatZeroK[2]) + (quatZeroK[3] * quatZeroK[3]));
              //if(quatZeroK[0] >= 0.0) quatZeroK[0] = sqrt(quatZeroK[0]);
              //else quatZeroK[0] = sqrt(quatZeroK[0] * -1.0) * -1.0;
              quatZeroK[0]=quatZero[0];

            }
          }
          // キャリブレーション完了後です。
          else{
            // add by nishi 2025.3.16
            // DMP の初期誤差だけ、補正します。
            #if defined(QUAT_CALIB)
              //double ans[4];
              //foxbot3::Kakezan(quatZeroK, quatRAW, ans);
              //foxbot3::Kakezan(ans, quatZero, quat);   // これをしたら、もとに戻る。
              // 下の積の処理だけで、OK みたい。 by nishi 2025.3.15
              foxbot3::Kakezan<double>(quatZeroK, quatRAW, quat);
              
            #else
              quat[0] = q0;
              quat[1] = q1;
              quat[2] = q2;
              quat[3] = q3;
            #endif
          }

          rc=true;

          //#if defined(QUAT_ANIMATION)
          #if defined(TEST_W0_X5)
            SERIAL_PORT.print(F("Q0:"));
            SERIAL_PORT.print(quat[0], 3);
            SERIAL_PORT.print(F("Q1:"));
            SERIAL_PORT.print(quat[1], 3);
            SERIAL_PORT.print(F(" Q2:"));
            SERIAL_PORT.print(quat[2], 3);
            SERIAL_PORT.print(F(" Q3:"));
            SERIAL_PORT.print(quat[3], 3);
            SERIAL_PORT.print(F(" Accuracy:"));
            SERIAL_PORT.println(data.Quat9.Data.Accuracy);
          #endif
        }
        else{
          //#define TEST_W0_X8
          #if defined(TEST_W0_X8)
            //SERIAL_PORT.print(F("#6 Q0:"));
            //SERIAL_PORT.print(q0, 3);
            //SERIAL_PORT.print(F(" Q1:"));
            //SERIAL_PORT.print(q1, 3);
            //SERIAL_PORT.print(F(" Q2:"));
            //SERIAL_PORT.print(q2, 3);
            //SERIAL_PORT.print(F(" Q3:"));
            //SERIAL_PORT.print(q3, 3);
            SERIAL_PORT.print(F(">#8 Accuracy:"));
            SERIAL_PORT.println(data.Quat9.Data.Accuracy);
          #endif
        }
      }
    #else
      // DMP 6 Fusion
      if((data.header & DMP_header_bitmap_Quat6) > 0){
        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30  -- X
        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30  -- Y
        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30  -- Z
        //double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W    こいつが、バグとの事。

        double q0=1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
        if(q0 >= 0.0) q0 = sqrt(q0);
        else q0 = sqrt(q0 * -1.0) * -1.0;

        if(!isnan(q0)){

          quatRAW[0] = q0;    // changed by nishi 2025.3.14
          quatRAW[1] = q1;    // changed by nishi 2025.3.14
          quatRAW[2] = q2;    // changed by nishi 2025.3.14
          quatRAW[3] = q3;    // changed by nishi 2025.3.14

          // キャリブレーション中です。
          if(calibratingD_f == 0){
            calibratingD++;
            if(calibratingD >= MPU_CALI_COUNT_DMP){
              if(calibratingD == MPU_CALI_COUNT_DMP){
                d[1]=0;
                d[2]=0;
                d[3]=0;
              }
              d[1] += quatRAW[1];             // Sum up 512 readings
              d[2] += quatRAW[2];             // Sum up 512 readings
              d[3] += quatRAW[3];             // Sum up 512 readings

              // キャリブレーションの終了回数に到達した。
              if(calibratingD >= MPU_CALI_COUNT_DMP*2-1){
                // DMP の初期誤差を計算する。
                // Z軸のみ補正します。by nishi 2025.3.17
                //quatZero[1] = d[1] / MPU_CALI_COUNT_DMP;
                quatZero[1] = 0.0;
                //quatZero[2] = d[2] / MPU_CALI_COUNT_DMP;
                quatZero[2] = 0.0;
                quatZero[3] = d[3] / MPU_CALI_COUNT_DMP; 

                calibratingD_f=1;
                calibratingD = 0;
              }
            }
            // キャリブレーションが終了した直後です。
            if (calibratingD_f == 1)
            {
              // DMP の誤差 quat の実数部を求める。
              //quatZero[0] = sqrt(1.0 - ((quatZero[1] * quatZero[1]) + (quatZero[2] * quatZero[2]) + (quatZero[3] * quatZero[3])));    //  -- W    こいつが、バグとの事。

              quatZero[0] =1.0 - ((quatZero[1] * quatZero[1]) + (quatZero[2] * quatZero[2]) + (quatZero[3] * quatZero[3]));
              if(quatZero[0] >= 0.0) quatZero[0] = sqrt(quatZero[0]);
              else quatZero[0] = sqrt(quatZero[0] * -1.0) * -1.0;

              // DMP 誤差 quat の補正(共役)  --> 誤差の分だけ、反対方向に回転させる。
              quatZeroK[1] = quatZero[1] * -1.0;
              quatZeroK[2] = quatZero[2] * -1.0;
              quatZeroK[3] = quatZero[3] * -1.0;
              //quatZeroK[0] =1.0 - ((quatZeroK[1] * quatZeroK[1]) + (quatZeroK[2] * quatZeroK[2]) + (quatZeroK[3] * quatZeroK[3]));
              //if(quatZeroK[0] >= 0.0) quatZeroK[0] = sqrt(quatZeroK[0]);
              //else quatZeroK[0] = sqrt(quatZeroK[0] * -1.0) * -1.0;
              quatZeroK[0]=quatZero[0];
            }
          }
          // キャリブレーション完了後です。
          else{
            // add by nishi 2025.3.16
            // DMP の初期誤差だけ、補正します。
            #if defined(QUAT_CALIB)
              //double ans[4];
              //foxbot3::Kakezan(quatZeroK, quatRAW, ans);
              //foxbot3::Kakezan(ans, quatZero, quat);   // これをしたら、もとに戻る。
              // 下の積の処理だけで、OK みたい。 by nishi 2025.3.15
              foxbot3::Kakezan<double>(quatZeroK, quatRAW, quat);

            #else
              quat[0] = q0;
              quat[1] = q1;
              quat[2] = q2;
              quat[3] = q3;
            #endif
          }

          rc=true;

          //#define TEST_W0_X2
          #if defined(TEST_W0_X2)
            SERIAL_PORT.print(F("Q0:"));
            SERIAL_PORT.print(q0, 3);
            SERIAL_PORT.print(F(" Q1:"));
            SERIAL_PORT.print(q1, 3);
            SERIAL_PORT.print(F(" Q2:"));
            SERIAL_PORT.print(q2, 3);
            SERIAL_PORT.print(F(" Q3:"));
            SERIAL_PORT.println(q3, 3);
          #endif
        }
      }
    #endif
  }
  return rc;
}
#endif

#if defined(USE_AGM_NISHI)
/*---------------
bool agm_get_adc()

  get acc, gyro and mag-calibed via dmp.
-----------------*/
bool cICM20948::agm_get_adc(){

  //#define TRACE_DMP_AGM1
  #if defined(TRACE_DMP_AGM1)
    SERIAL_PORT.println("agm_get_adc() start");
  #endif

  /*
  * add 9 axis fusion Quarternion with DMP3
  */
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.

  bool rc=false;
  icm_20948_DMP_data_t data;

  int cnt=0;
  //static int32_t d[4];
  // changed by nishi 2025.3.14
  //static double d[4];
  
  //static byte f=0;
  //static double q[4]={1.0, 0.0, 0.0, 0.0};

  agm_rcv_f[0]=agm_rcv_f[1]=agm_rcv_f[1]=false;

  magADC_BD[0]=magADC_BD[1]=magADC_BD[2]=0.0;

  //#define DISP_DMP_AGM_6_x

  //#define QUAT_CALIB
  while(1){
    myICM.readDMPdataFromFIFO(&data);
    if ((myICM.status != ICM_20948_Stat_Ok) && (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
      break;

    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    #if defined(TRACE_DMP_AGM_11)
      SERIAL_PORT.println("dmp_get_adc() #3");
    #endif

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // Check for orientation data (Quat6)
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      #if defined(DISP_DMP_AGM_6_x)
        SERIAL_PORT.print(F("Q1:"));
        SERIAL_PORT.print(q1, 3);
        SERIAL_PORT.print(F(" Q2:"));
        SERIAL_PORT.print(q2, 3);
        SERIAL_PORT.print(F(" Q3:"));
        SERIAL_PORT.println(q3, 3);
      #endif
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      accADC[0]=accRAW[0] = data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      accADC[1]=accRAW[1] = data.Raw_Accel.Data.Y;
      accADC[2]=accRAW[2] = data.Raw_Accel.Data.Z;

      #if defined(DISP_DMP_AGM_6_x)
      SERIAL_PORT.print(F("accRAW:"));
      SERIAL_PORT.print(accRAW[0]);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(accRAW[1]);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(accRAW[2]);
      #endif
      agm_rcv_f[0]=true;
      cnt++;
      acc_common();
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      gyroRAW[0] = data.Raw_Gyro.Data.X; // Extract the raw gyro data
      gyroRAW[1] = data.Raw_Gyro.Data.Y;
      gyroRAW[2] = data.Raw_Gyro.Data.Z;

      #if defined(DISP_DMP_AGM_6_x)
      SERIAL_PORT.print(F("gyroRAW: "));
      SERIAL_PORT.print(gyroRAW[0]);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(gyroRAW[1]);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(gyroRAW[2]);
      #endif
      agm_rcv_f[1]=true;
      cnt++;
      GYRO_ORIENTATION( gyroRAW[0], gyroRAW[1], gyroRAW[2]);
      gyro_common();
    }      

    if ((data.header & DMP_header_bitmap_Compass) > 0){ // Check for Compass
      float x = (float)data.Compass.Data.X; // Extract the compass data
      float y = (float)data.Compass.Data.Y;
      float z = (float)data.Compass.Data.Z;

      //#define DISP_DMP6_x2
      #if defined(DISP_DMP_AGM_6_x)
        SERIAL_PORT.print(F("Compass:"));
        SERIAL_PORT.print(x);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.print(y);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.println(z);
      #endif
    }

    if ((data.header & DMP_header_bitmap_Compass_Calibr) > 0){ // Check the packet contains Accel data
      // 参照は、
      // /home/nishi/Arduino/lib-ros2-test/SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_DMP.h
      magRAW32[0] = data.Compass_Calibr.Data.X;
      magRAW32[1] = data.Compass_Calibr.Data.Y;
      magRAW32[2] = data.Compass_Calibr.Data.Z;

      //magADC_BD[0] = (float)data.Compass_Calibr.Data.X * mRes;
      //magADC_BD[1] = (float)data.Compass_Calibr.Data.Y * mRes;
      //magADC_BD[2] = (float)data.Compass_Calibr.Data.Z * mRes;


      uint16_t Compass_Accuracy = data.Compass_Accuracy;

      #if defined(DISP_DMP_AGM_6_x)
        // #4 Mag_Cali:461537.281 ,-284590.219 ,-660603.000
        SERIAL_PORT.print(F("magRAW32:"));
        SERIAL_PORT.print(magRAW32[0]);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.print(magRAW32[1]);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.println(magRAW32[2]);
        //SERIAL_PORT.print(F(" Accuracy:"));
        //SERIAL_PORT.println(Compass_Accuracy);
      #endif

      agm_rcv_f[3]=true;
      cnt++;

    }
  }
  if((agm_rcv_f[0] & agm_rcv_f[1]) == true ){
    rc=true;
  }
  #if defined(DISP_DMP_AGM_6_x)
    SERIAL_PORT.print(F(" cnt:"));
    SERIAL_PORT.println(cnt);
  #endif
  agm_common();
  
  return rc;
}

void cICM20948::agm_common(){
	static long long m32[3];  // 4900.0 / 0.15 = 32666.6  , 32,666.6*100 = 32,66,666.6
  // mag cali comming
  if(agm_rcv_f[3]==true){
    if(calibratingM_f == 0){
      calibratingM++;
      if(calibratingM >= MPU_CALI_COUNT_MAG){
  
        if(calibratingM == MPU_CALI_COUNT_ACC){
          m32[0]=0;
          m32[1]=0;
          m32[2]=0;
        }
        m32[0] += magRAW32[0];   // Sum up 512 readings
        m32[1] += magRAW32[1];   // Sum up 512 readings
        m32[2] += magRAW32[2];   // Sum up 512 readings
  
        if(calibratingM >= (MPU_CALI_COUNT_MAG*2 - 1)){
          magZero32[0] = m32[0] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
          magZero32[1] = m32[1] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
          magZero32[2] = m32[2] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
  
          // test by nishi 2025.3.25
          //mag_offsets[0] = (float)magZero[0] * mRes * -1.0; 
          //mag_offsets[1] = (float)magZero[1] * mRes * -1.0;
          //mag_offsets[2] = (float)magZero[2] * mRes * -1.0;
  
          //mag_offsets[0] = (float)magZero[0];
          //mag_offsets[1] = (float)magZero[1];
          //mag_offsets[2] = (float)magZero[2];
  
          calibratingM_f=1;
          calibratingM=0;
        }
      }
    }
    if(calibratingM_f == 1){
      float x = (float)magRAW32[0] * mRes - mag_offsets[0];  // scaled data [uT]
      float y = (float)magRAW32[1] * mRes - mag_offsets[1];  // scaled data [uT]
      float z = (float)magRAW32[2] * mRes - mag_offsets[2];  // scaled data [uT]
  
      //float x = (float)magRAW[0] - mag_offsets[0];  // scaled data [uT]
      //float y = (float)magRAW[1] - mag_offsets[1];  // scaled data [uT]
      //float z = (float)magRAW[2] - mag_offsets[2];  // scaled data [uT]
  
  
      // Apply mag soft iron error compensation
      magADC_BD[0] = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
      magADC_BD[1] = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
      magADC_BD[2] = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
  
      //magADC_BD[0] = (float)magRAW[0] * mRes;
      //magADC_BD[1] = (float)magRAW[1] * mRes;
      //magADC_BD[2] = (float)magRAW[2] * mRes;
  
      //magADC_BD[0] = x;
      //magADC_BD[1] = y;
      //magADC_BD[2] = z;
      // normalize
      //double magIMZero = sqrt(magADC_BD[0] * magADC_BD[0] + magADC_BD[1] * magADC_BD[1] + magADC_BD[2] * magADC_BD[2]);
      //magADC_BD[0] /= magIMZero;
      //magADC_BD[1] /= magIMZero;
      //magADC_BD[2] /= magIMZero;
    
      #if defined(USE_AGM_NISHI_DEBUG_X8)
        SERIAL_PORT.print(F("mag_offsets: "));
        SERIAL_PORT.print(mag_offsets[0], 3);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.print(mag_offsets[1], 3);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.println(mag_offsets[2], 3);
        // mag_offsets: 90.600 ,48.900 ,55.650
        //SERIAL_PORT.print(F(" ,"));
        //SERIAL_PORT.println(accuracy);
      #endif
  
    }
  
    magADC32[0] =  magRAW32[0];  //
    magADC32[1] =  magRAW32[1];  //
    magADC32[2] =  magRAW32[2];  //
  
    // Sensitivity Scale Factor = 0.15 add by nishi 2021.11.4
    //  Full Range +_4900 [uT]
    //magADC_BD[0] =  (float)(magRAW[0] - magZero[0]) * MAG_UT_LSB;  // [uT]
    //magADC_BD[1] =  (float)(magRAW[1] - magZero[1]) * MAG_UT_LSB;  // [uT]
    //magADC_BD[2] =  (float)(magRAW[2] - magZero[2]) * MAG_UT_LSB;  // [uT]
  
    //#define USE_AGM_NISHI_DEBUG_X7
    #if defined(USE_AGM_NISHI_DEBUG_X7)
      SERIAL_PORT.print(F("magADC32: "));
      SERIAL_PORT.print(magADC32[0]);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(magADC32[1]);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(magADC32[2]);
      //SERIAL_PORT.print(F(" ,"));
      //SERIAL_PORT.println(accuracy);
    #endif

  }
}

#endif  

void cICM20948::acc_cali_start()
{
	//calibratingA = MPU_CALI_COUNT;
  calibratingA_f = 0;
  calibratingA = 0; 
}

bool cICM20948::acc_cali_get_done()
{
	if( calibratingA_f !=0 ) return true;
	else                    return false;
}

bool cICM20948::gyro_cali_get_done()
{
	if( calibratingG_f != 0 ) return true;
	else                    return false;
}

bool cICM20948::mag_cali_get_done()
{
	if( calibratingM_f != 0 ) return true;
	else                    return false;
}


//#define USE_DMP_INIT_CUST
#if defined(USE_DMP_NISHI) && defined(USE_DMP_INIT_CUST)
///
// initializeDMP is a weak function. Let's overwrite it so we can increase the sample rate
ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful
  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = enableDMP(false); if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  //myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  // add by nishi 2025.3.30
  #if defined(USE_ACC_2G)
    myFSS.a = gpm2;  // gpm2
  #elif defined(USE_ACC_4G)
    myFSS.a = gpm4;  // gpm4
  #else
    myFSS.a = gpm8;  // gpm8
  #endif
                   

  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // add by nishi start
    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    //myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)   original
    //myDLPcfg.a = acc_d246bw_n265bw;     // acc_d246bw_n265bw  -> 7. NG    - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
    //myDLPcfg.a = acc_d111bw4_n136bw;   // acc_d111bw4_n136bw -> 6. OK!
    //myDLPcfg.a = acc_d50bw4_n68bw8; // acc_d50bw4_n68bw8 -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
    //myDLPcfg.a = acc_d23bw9_n34bw4;  // acc_d23bw9_n34bw4 -> 4.  so so
    //myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Good! ただし #3のばあい
    //myDLPcfg.a =  acc_d5bw7_n8bw3;  // acc_d5bw7_n8bw3   - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz  -> 2 NG
                                    // acc_d473bw_n499bw   original  1. soso

    #if defined(USE_FOXBOT)
      //myDLPcfg.a = acc_d111bw4_n136bw;   // acc_d111bw4_n136bw -> 6. OK!
      myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Good! ただし #3のばあい
      #else
      //myDLPcfg.a = acc_d246bw_n265bw;     // acc_d246bw_n265bw  -> 7. NG   - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
      //myDLPcfg.a = acc_d111bw4_n136bw;   // acc_d111bw4_n136bw -> 6. OK!
      //myDLPcfg.a = acc_d50bw4_n68bw8; // acc_d50bw4_n68bw8 -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
      myDLPcfg.a =  acc_d11bw5_n17bw;  // acc_d11bw5_n17bw   -> 3. Good! ただし #3のばあい
    #endif

    //myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)  original
                                      // gyr_d196bw6_n229bw8
    //myDLPcfg.g =  gyr_d151bw8_n187bw6;  // gyr_d151bw8_n187bw6  -> 7. NG
    //myDLPcfg.g = gyr_d119bw5_n154bw3;   // gyr_d119bw5_n154bw3  -> 6. OK 
    //myDLPcfg.g =  gyr_d51bw2_n73bw3;  // gyr_d51bw2_n73bw3  -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
    //myDLPcfg.g =  gyr_d23bw9_n35bw9;    // gyr_d23bw9_n35bw9   -> 4.  so so
    //myDLPcfg.g = gyr_d11bw6_n17bw8;   // gyr_d11bw6_n17bw8  -> 3. Good!  ただし #3のばあい
    //myDLPcfg.g = gyr_d5bw7_n8bw9;     // gyr_d5bw7_n8bw9  -> 2. NG
                                      // gyr_d361bw4_n376bw5  original  1. so so
    #if defined(USE_FOXBOT)
      //myDLPcfg.g = gyr_d119bw5_n154bw3;   // gyr_d119bw5_n154bw3  -> 6. OK 
      //myDLPcfg.g =  gyr_d51bw2_n73bw3;  // gyr_d51bw2_n73bw3  -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
      myDLPcfg.g = gyr_d11bw6_n17bw8;   // gyr_d11bw6_n17bw8  -> 3. Good!  ただし #3のばあい
    #else
      //myDLPcfg.g =  gyr_d151bw8_n187bw6;  // gyr_d151bw8_n187bw6  -> 7.  NG
      //myDLPcfg.g = gyr_d119bw5_n154bw3;   // gyr_d119bw5_n154bw3  -> 6. OK 
      //myDLPcfg.g =  gyr_d51bw2_n73bw3;  // gyr_d51bw2_n73bw3  -> 5. Good! ただし #2 のばあい。Madgwick betaDef 0.1f だとNG
      myDLPcfg.g = gyr_d11bw6_n17bw8;   // gyr_d11bw6_n17bw8  -> 3. Good!  ただし #3のばあい
    #endif

    result = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg); if (result > worstResult) worstResult = result;
    //myICM.setDLPFcfg(ICM_20948_Internal_Acc , myDLPcfg);
    //if (myICM.status != ICM_20948_Stat_Ok){
    //  SERIAL_PORT.print(F("setDLPcfg returned: "));
    //  SERIAL_PORT.println(myICM.statusString());
    //  rc = false; // add by nishi 2022.4.23
    //}    
    result = enableDLPF(ICM_20948_Internal_Acc, true); if (result > worstResult) worstResult = result;

  // add by nishi end

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.g = 4; // 225Hz
  //mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  // add by nishi 2025.3.30
  #if defined(USE_ACC_2G)
    const unsigned char accScale[4] = {0x01, 0x00, 0x00, 0x00};
  #elif defined(USE_ACC_4G)
    const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  #else
    const unsigned char accScale[4] = {0x08, 0x00, 0x00, 0x00};
  #endif
  //const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g

  // add by nishi 2025.3.30
  #if defined(USE_ACC_2G)
    const unsigned char accScale2[4] = {0x00, 0x01, 0x00, 0x00};
  #elif defined(USE_ACC_4G)
    const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  #else
    const unsigned char accScale2[4] = {0x00, 0x08, 0x00, 0x00};
  #endif
  //const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(19, 3); if (result > worstResult) worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)
  //result = setGyroSF(4, 3); if (result > worstResult) worstResult = result; // 4 = 225Hz (see above), 3 = 2000dps (see above)
  //result = setGyroSF(8, 3); if (result > worstResult) worstResult = result;   // 8 = 112Hz  (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}
#endif
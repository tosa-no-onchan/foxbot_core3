/*
* BNO086.cpp
*/
#include <Arduino.h>
#include <SPI.h>

#include "BNO086.h"


//#define SPI_CS_PIN          BDPIN_SPI_CS_IMU
                            
#define ICM20948_ADDRESS    0xEA
#define MPU_CALI_COUNT      512


//#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  = -X; accADC[ROLL]  =  Y; accADC[YAW]  =   Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

//#define DEBUG_M

float cBNO086::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

cBNO086::cBNO086()
{
	calibratingG = 0;
	calibratingA = 0;
  calibratingM = 0;
  bConnected   = false;

  // ICM20948
  // Full-Scale Range
  //  ACCEL_FS=0  -> ±2 [G]
  //  ACCEL_FS=1  -> ±4 [G]
  //  ACCEL_FS=2  -> ±8 [G]
  //  ACCEL_FS=3  -> ±16 [G]
  // Sensitivity Scale Factor
  //  ACCEL_FS=0 -> 16,384 [LSB/g]
  //  ACCEL_FS=1 -> 8,192 [LSB/g]
  //  ACCEL_FS=2 -> 4,096 [LSB/g]
  //  ACCEL_FS=3 -> 2,048 [LSB/g]
  // initial Tolerance
  //  Component-level ±0.5[%]
  // ZERO-G OUTPUT
  //  Initial Tolerance Board-level, all axes ±50 [mg]
  //aRes = 8.0/32768.0;      // 8g    16bit -> 8G

  // ICM20948

  // 2G       16384.0
  // 4G       8192.0
  // 8G       4096.0
  // 16G      2048.0

  //aRes = 9.80665/16384.0;    // 2g
  //aRes = 9.80665/8192.0;     // 4g
  //aRes = 9.80665/4096.0;     // 8g
  //aRes = 9.80665/2048.0;     // 16g
  //aRes = 9.80665/ACC_MAX_G; 

  // Gyro
  // 2000[dps]  -> 16.4[LSB/dps] -> gx/16.4 -> [dps]
  //gRes = 1.0/16.4;   // 2000dps
  // org setting
  // gRes = 2000.0/32768.0;   // 2000dps   -> 0.06103515625
  // gx = (float)SEN.gyroADC[0]*gRes;

  //mRes = 0.15; // Sensitivity Scale Factor = 0.15

  // ACC Z 軸の Zero 調整
  // Z 軸の停止時の中心が、 0 になるように調整します。
  zero_off = ACC_ZERO_OFF;

  #if defined(USE_DMP_NISHI)
    //begin_update_rate = 30;
    //begin_update_rate = 50;
    //begin_update_rate = 333;  // update_motor() でうまく行かない。 main.cpp TEST2 では、OK
    //begin_update_rate = 250;  // update_motor() でうまく行かない。 main.cpp TEST2 では、OK
    //begin_update_rate = 200;  // OK 実測 199[Hz] [ 73472][E][Wire.cpp:513] ずっと動かすと、requestFrom(): i2cRead returned Error 263 cBNO086::update(): #1 sensor was reset 
    //begin_update_rate = 180;  // OK 実測 180[Hz] OK and NG
    //begin_update_rate = 170;  // OK 
    begin_update_rate = 150;  // OK 安全マージン
  #elif defined(USE_AG_NISHI)
    begin_update_rate = 200;  // 実測 17 - 30[Hz]
    //begin_update_rate = 333;  // NG
    //begin_update_rate = 500;  // NG
  #elif defined(USE_MAG)
    begin_update_rate = 100;
  #else
    //begin_update_rate = 333;  // NG
    //begin_update_rate = 300;  // NG
    //begin_update_rate = 200;    // 実測 108 - 130[Hz]
    begin_update_rate = 150;    // 
    //begin_update_rate = 100;
    //begin_update_rate = 50;
  #endif
}


bool cBNO086::begin()
{
  int cnt;
  uint8_t data;

  #if defined(USE_WIRE)
    //Wire.begin();
    // Wire.begin(sda,scl)
    pinMode(21,PULLUP);
    pinMode(22,PULLUP);
    Wire.begin(21,22);
  #else
    // test by nishi MOSI(23) Pull UP 2023.12.13
    //pinMode(23,INPUT_PULLUP);
    #if defined(BOARD_ESP32)
      SPI.begin(18,19,23,BNO08X_CS);    // SCLK,MISO,MOSI,CS
    #elif defined(BOARD_PICO32)
      SPI.begin(18,36,26,BNO08X_CS);    // SCLK,MISO,MOSI,CS
    #endif
  #endif


  bool initialized = false;
  bConnected=false;
  
  //cnt = 2;
  cnt = 4;  // changed by nishi 2023.3.4
  while (!initialized)
  {
    bool rc;
    #if defined(USE_WIRE)
      //rc=myIMU.begin();  // Setup without INT/RST control (Not Recommended)
      rc=myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST);
    #else
      // boolean BNO08x::beginSPI(uint8_t user_CSPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed, SPIClass &spiPort)
      //rc=myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST,3000000,MPU_SPI);
      rc=myIMU.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST,1000000,MPU_SPI);
    #endif

    //SERIAL_PORT.println("cBNO086::begin(): #2");

    // add by nisshi 2024.2.17
    //myICM.swReset();
    //delay(250);

    if (rc == false)
    {
      SERIAL_PORT.print(F("Initialization of the sensor returned error"));
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

  #if defined(USE_WIRE)
    Wire.setClock(400000); //Increase I2C data rate to 400kHz
    //Wire.setClock(100000); //Increase I2C data rate to 100kHz
    //Wire.setClock(200000); //Increase I2C data rate to 200kHz
  #endif

  if(initialized == true){
    if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_GYRO || SH2_CAL_MAG) == true) { // all three sensors
    //if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_GYRO) == true) { // 
    //if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_MAG) == true) { // Default settings
    //if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL) == true) { // only accel
      Serial.println(F("Calibration Command Sent Successfully"));
    } 
    else {
      Serial.println("Could not send Calibration Command.");
      initialized == false;
    }
  }


  if(initialized == true){
    bConnected=init();
  }

  return bConnected;
}

bool cBNO086::init( void ){
  bool rc=true;

  SERIAL_PORT.println("cBNO086::int(): #1");

  #if !defined(USE_DMP_NISHI)
    #if defined(USE_ACC_NISHI) || defined(USE_AG_NISHI)  || defined(USE_AGM_NISHI)
      // ms で、間隔を指定できるみたい。200[Hz] -> 5[ms]
      //    3[ms] -> 333[Hz]
      //rc =myIMU.enableAccelerometer(5);
      //rc =myIMU.enableAccelerometer(3);
      rc =myIMU.enableAccelerometer(1);
      if(rc ==false){
        SERIAL_PORT.print("cBNO086::init(): #1 Could not enable accelerometer");
        return rc;
      }
    #endif

    #if defined(USE_GRYO_NISHI) || defined(USE_AG_NISHI)  || defined(USE_AGM_NISHI)
      // ms で、間隔を指定できるみたい。200[Hz] -> 5[ms]
      //    3[ms] -> 333[Hz]
      //rc=myIMU.enableGyro(3);
      //rc=myIMU.enableUncalibratedGyro(3);
      rc=myIMU.enableGyro(1);
      if (rc == false) {
        SERIAL_PORT.print("cBNO086::init(): #2 Could not enable  gyro");
        return rc;
      }
    #endif
    rc=myIMU.enableGameRotationVector(1);
    if (rc == false) {
      Serial.println("cBNO086::init(): #3 Could not enable game rotation vector");
      return rc;
    }
  #endif

  #if defined(USE_DMP_NISHI)
    //delay(10);
    // ms で、間隔を指定できるみたい。50[Hz] -> 20[ms]
    // ms で、間隔を指定できるみたい。400[Hz] -> 2.5[ms]
    // ms で、間隔を指定できるみたい。333[Hz] -> 3[ms]
    // 2.2.4 RotationVector -> DMP9
    rc = myIMU.enableRotationVector(1);
    //rc = myIMU.enableRotationVector();  // 指定しないと NG
    // 2.2.6 Gyro rotation Vector   --> low atency しかし、無い
    if(rc == false){
      SERIAL_PORT.print("cBNO086::init(): #3 Could not enable rotation vector");
      return rc;
    }
  #endif

  #if defined(USE_MAG) || defined(USE_AGM_NISHI)
    rc = myIMU.enableMagnetometer(10);
    if (rc == false) {
      SERIAL_PORT.print("cBNO086::init(): #4 Could not enable magnetometer");
      return rc;
    }
  #endif

  delay(100); // This delay allows enough time for the BNO086 to accept the new 
  return rc;
}

bool cBNO086::update( void ){
  if (myIMU.wasReset()) {
    //#if defined(TRACE_DMP1)
      SERIAL_PORT.println("cBNO086::update(): #1 sensor was reset ");
    //#endif
    init();
    //delay(100);
  }
  // check data comming?
  bool rc=myIMU.getSensorEvent();
  //SERIAL_PORT.print("cBNO086::update(): #99 rc:");
  //SERIAL_PORT.println(rc);
  return rc;
}

void cBNO086::gyro_init( void ){
	uint8_t i;
	for( i=0; i<3; i++ ){
    gyroADC[i]  = 0;
		gyroZero[i] = 0;
		gyroRAW[i]  = 0;
    gyroADC_BD[i] = 0.0;
		gyroZero_BD[i] = 0.0;
	}
  calibratingG_f = 0;
  calibratingG = 0; 
	//calibratingG = MPU_CALI_COUNT;
}

bool cBNO086::gyro_get_adc( void ){
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;

  bool rc=false;

  //#define USE_GYRO_DEBUG_X
  #if defined(USE_GYRO_DEBUG_X)
    SERIAL_PORT.print("cBNO086::gyro_get_adc(): #1 ID:");
    SERIAL_PORT.println(myIMU.getSensorEventID());
  #endif


  // is it the correct sensor data we want?
  if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {

    x = myIMU.getGyroX();
    y = myIMU.getGyroY();
    z = myIMU.getGyroZ();       // こいつ、取れていないみたい。ノイズだけ入る。

    uint8_t accuracy = myIMU.getGyroAccuracy();

    // dgree per sec みたい

    gyroADC_BD[0] = x;
    gyroADC_BD[1] = y;
    gyroADC_BD[2] = z;

    //#define USE_GYRO_DEBUG_X2
    #if defined(USE_GYRO_DEBUG_X2)
      SERIAL_PORT.print("gyroADC_BD:");
      SERIAL_PORT.print(gyroADC_BD[0]);
      SERIAL_PORT.print(",");
      SERIAL_PORT.print(gyroADC_BD[1]);
      SERIAL_PORT.print(",");
      SERIAL_PORT.print(gyroADC_BD[2]);
      SERIAL_PORT.print(" accuracy:");
      SERIAL_PORT.print(accuracy);
      SERIAL_PORT.print("\n");

      //gyroADC_BD:0.00,0.00,0.00
      //gyroADC_BD:-0.02,4.95,0.00    ->  ラジアンでは無い
      //gyroADC_BD:-0.03,4.93,0.00
      //gyroADC_BD:-0.03,4.92,0.00
      //gyroADC_BD:0.00,0.00,0.00
      //gyroADC_BD:-0.02,4.94,0.00
      //gyroADC_BD:0.00,0.00,0.00
      //gyroADC_BD:0.00,0.00,-0.00
      //gyroADC_BD:0.00,0.00,0.00
      //gyroADC_BD:0.00,0.00,0.01
      //gyroADC_BD:0.00,0.00,0.00
      //gyroADC_BD:-0.01,-0.00,-0.00

      #endif

  	//gyroRAW[0] = x = myICM.agmt.gyr.axes.x;
  	//gyroRAW[1] = y = myICM.agmt.gyr.axes.y;
  	//gyroRAW[2] = z = myICM.agmt.gyr.axes.z;

  	//GYRO_ORIENTATION( x, y,z );
    gyro_common();
    rc=true;
  }
  else if(myIMU.getSensorEventID()==SENSOR_REPORTID_UNCALIBRATED_GYRO){
    return false;
    x = myIMU.getUncalibratedGyroX();
    y = myIMU.getUncalibratedGyroY();
    z = myIMU.getUncalibratedGyroZ();

    //uint8_t accuracy = myIMU.getGyroAccuracy();

    gyroADC_BD[0] = x;
    gyroADC_BD[1] = y;
    gyroADC_BD[2] = z;

    //#define USE_GYRO_DEBUG_X3
    #if defined(USE_GYRO_DEBUG_X3)
      SERIAL_PORT.print("gyroUn_callib_ADC_BD:");
      //SERIAL_PORT.print(gyroADC_BD[0]);
      //SERIAL_PORT.print(",");
      //SERIAL_PORT.print(gyroADC_BD[1]);
      //SERIAL_PORT.print(",");
      SERIAL_PORT.print(gyroADC_BD[2]);
      //SERIAL_PORT.print(" accuracy:");
      //SERIAL_PORT.print(accuracy);
      SERIAL_PORT.print("\n");

      //gyroUn_callib_ADC_BD:-0.00,0.00,-59.00   --> でかいノイズ
      //gyroUn_callib_ADC_BD:0.01,0.00,0.00
      //gyroUn_callib_ADC_BD:0.01,-0.01,0.00
      //gyroUn_callib_ADC_BD:0.00,0.00,0.00
      //gyroUn_callib_ADC_BD:0.00,0.00,0.00
      //gyroUn_callib_ADC_BD:-0.01,-0.01,0.00
      //gyroUn_callib_ADC_BD:-0.01,-0.01,0.00
      //gyroUn_callib_ADC_BD:-0.00,-0.00,-0.00
      //gyroUn_callib_ADC_BD:0.00,-0.00,-0.01

    #endif

  	//gyroRAW[0] = x = myICM.agmt.gyr.axes.x;
  	//gyroRAW[1] = y = myICM.agmt.gyr.axes.y;
  	//gyroRAW[2] = z = myICM.agmt.gyr.axes.z;

  	//GYRO_ORIENTATION( x, y,z );
    gyro_common();
    rc=true;
  }
  return rc;
}

void cBNO086::gyro_cali_start(){
	//calibratingG = MPU_CALI_COUNT;
  calibratingG_f = 0;
  calibratingG = 0; 
}

void cBNO086::gyro_common(){
	static float g[3];

  if(calibratingG_f == 0){
    calibratingG++;
    if(calibratingG >= MPU_CALI_COUNT_GYRO){
      if(calibratingG == MPU_CALI_COUNT_GYRO){
        g[0]=0.0;
        g[1]=0.0;
        g[2]=0.0;
      }
			g[0] += gyroADC_BD[0];             // Sum up 512 readings
			g[1] += gyroADC_BD[1];             // Sum up 512 readings
			g[2] += gyroADC_BD[2];             // Sum up 512 readings

      if(calibratingG >= MPU_CALI_COUNT_GYRO * 2){
        gyroZero_BD[0] = g[0] / MPU_CALI_COUNT_GYRO;
        gyroZero_BD[1] = g[1] / MPU_CALI_COUNT_GYRO;
        gyroZero_BD[2] = g[2] / MPU_CALI_COUNT_GYRO;
        //gyroZeroSum=gyroZero[0]+gyroZero[1]+gyroZero[2];
        calibratingG_f=1;
        calibratingG=0;
        // test by nishi 2025.3.25
        //gyroZero[0]=gyroZero[1]=gyroZero[2]=0;
      }
    }
  }
  //gyroADC[0] -= gyroZero[0];
  //gyroADC[1] -= gyroZero[1];
  //gyroADC[2] -= gyroZero[2];

  // GYRO 0 noise cut off
  // MadgwickAHRS.cpp and FusionAhrs.c では、gyro値 = 0.0 だとアクセスエラーと
  // 判定するみたいだ。 by nishi 2022.5.12
  //for (int axis = 0; axis < 3; axis++){
  //  if (abs(gyroADC[axis]) <= GYRO_NOISE_CUT_OFF){
  //    //gyroADC[axis] = 0;
  //  }
  //}
}

void cBNO086::acc_init( void ){
  uint8_t i;
  for( i=0; i<3; i++ ){
    accADC[i]   = 0;
		accZero[i]  = 0;
    accRAW[i]   = 0;
    accADC_BD[i] = 0.0;
		accZero_BD[i]  = 0.0;
  }

  calibratingA_f = 0;
  calibratingA = 0;    // add by nishi 2022.1.23
  accIMZero=0.0;
}

bool cBNO086::acc_get_adc( void ){
	//int16_t x = 0;
	//int16_t y = 0;
	//int16_t z = 0;
  //uint8_t rawADC[6];

  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  bool rc=false;

  #if defined(USE_ACC_DEBUG_X)
    SERIAL_PORT.println("cBNO086::acc_get_adc(): #1");
  #endif

  if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {

    #if defined(USE_ACC_DEBUG_X)
      SERIAL_PORT.println("cBNO086::acc_get_adc(): #2");
    #endif

    x = myIMU.getAccelX();
    //x = myIMU.getAccelX()* -1.0;
    y = myIMU.getAccelY();
    //y = myIMU.getAccelY() * -1.0;
    z = myIMU.getAccelZ();
    //z = myIMU.getAccelZ() * -1.0; // - にすると、天地逆になる。

    uint8_t accuracy = myIMU.getAccelAccuracy();

    //#define USE_ACC_DEBUG_X2
    #if defined(USE_ACC_DEBUG_X2)
      SERIAL_PORT.print("acc:");
      SERIAL_PORT.print(x);
      SERIAL_PORT.print(" ,");
      SERIAL_PORT.print(y);
      SERIAL_PORT.print(" ,");
      SERIAL_PORT.print(z);
      //SERIAL_PORT.print(" accuracy:");
      //SERIAL_PORT.print(accuracy);
      SERIAL_PORT.print("\n");

      //acc:0.00 ,0.00 ,0.00
      //acc:-0.02 ,-0.07 ,9.88
      //acc:-0.01 ,-0.01 ,9.90
      //acc:0.00 ,0.00 ,0.00

      //acc:0.00 ,0.13 ,0.00 accuracy:2
      //acc:0.02 ,0.05 ,9.89 accuracy:2
      //acc:-0.05 ,0.05 ,0.00 accuracy:3
      //acc:50.01 ,-0.00 ,0.01 accuracy:2
      //acc:-0.03 ,0.04 ,0.00 accuracy:1
      //acc:87.01 ,0.00 ,0.11 accuracy:2
      //acc:-0.05 ,0.09 ,0.00 accuracy:2
      //acc:0.01 ,0.04 ,9.91 accuracy:2
      //acc:0.04 ,0.00 ,0.00 accuracy:1
      //acc:85.01 ,0.04 ,0.06 accuracy:3
      //acc:123.01 ,0.01 ,0.00 accuracy:2
      //acc:-0.04 ,0.09 ,9.90 accuracy:2
      //acc:-0.07 ,0.11 ,0.00 accuracy:3
      //acc:48.01 ,-0.03 ,-0.02 accuracy:2
      //acc:-0.03 ,0.11 ,9.97 accuracy:3
      //acc:118.01 ,-0.01 ,-0.01 accuracy:2

    #endif    

    //if(x==0.0 && y==0.0 && x==0.0)
    if(accuracy < 2 || (x==0.0 && y==0.0 && x==0.0))
    //if(accuracy == 0)
      return false;

    //#define USE_ACC_DEBUG_X3
    #if defined(USE_ACC_DEBUG_X3)
      SERIAL_PORT.print("acc:");
      SERIAL_PORT.print(x);
      SERIAL_PORT.print(" ,");
      SERIAL_PORT.print(y);
      SERIAL_PORT.print(" ,");
      SERIAL_PORT.print(z);
      //SERIAL_PORT.print(" accuracy:");
      //SERIAL_PORT.print(accuracy);
      SERIAL_PORT.print("\n");

      //acc:0.00 ,0.00 ,0.00
      //acc:-0.02 ,-0.07 ,9.88
      //acc:-0.01 ,-0.01 ,9.90
      //acc:0.00 ,0.00 ,0.00

      //acc:0.00 ,0.13 ,0.00 accuracy:2
      //acc:0.02 ,0.05 ,9.89 accuracy:2
      //acc:-0.05 ,0.05 ,0.00 accuracy:3
      //acc:50.01 ,-0.00 ,0.01 accuracy:2
      //acc:-0.03 ,0.04 ,0.00 accuracy:1
      //acc:87.01 ,0.00 ,0.11 accuracy:2
      //acc:-0.05 ,0.09 ,0.00 accuracy:2
      //acc:0.01 ,0.04 ,9.91 accuracy:2
      //acc:0.04 ,0.00 ,0.00 accuracy:1
      //acc:85.01 ,0.04 ,0.06 accuracy:3
      //acc:123.01 ,0.01 ,0.00 accuracy:2
      //acc:-0.04 ,0.09 ,9.90 accuracy:2
      //acc:-0.07 ,0.11 ,0.00 accuracy:3
      //acc:48.01 ,-0.03 ,-0.02 accuracy:2
      //acc:-0.03 ,0.11 ,9.97 accuracy:3
      //acc:118.01 ,-0.01 ,-0.01 accuracy:2

    #endif    


    accADC_BD[0] = x;
    accADC_BD[1] = y;
    accADC_BD[2] = z;

    //accRAW[0] = x = myICM.agmt.acc.axes.x;
    //accRAW[1] = y = myICM.agmt.acc.axes.y;
    //accRAW[2] = z = myICM.agmt.acc.axes.z;

    // レゾルーションを掛けた値?  agmt.fss.a を元に、G に変換した値
    //switch (agmt.fss.a){
    //case 0: -> (((float)axis_val) / 16.384);
    //case 1: -> (((float)axis_val) / 8.192);
    //case 2: -> (((float)axis_val) / 4.096);
    //case 3: -> (((float)axis_val) / 2.048);
    //accADC_BD[0] = myICM.accX();
    //accADC_BD[1] = myICM.accY();
    //accADC_BD[2] = myICM.accZ();

    //ACC_ORIENTATION( x,	y, z );
    acc_common();
    rc=true;
  }
  return rc;
}

/*
* get Average of Acc value
* from start to 512 
* get Average for each acc x,y,z
* but z is set to zero;
*/
void cBNO086::acc_common(){
	static float a[3];

  if(calibratingA_f == 0){
    calibratingA++;
    if(calibratingA >= MPU_CALI_COUNT_ACC){
      // Z 軸のノイズを取り除く
      //if(abs(accADC[2] - ACC_1G) > ACC_ZERO_Z_OVER){
      //  calibratingA--;
      //  return;
      //}
      if(calibratingA == MPU_CALI_COUNT_ACC){
        a[0]=0.0;
        a[1]=0.0;
        a[2]=0.0;
      }
			a[0] += accADC_BD[0];             // Sum up 512 readings
			a[1] += accADC_BD[1];             // Sum up 512 readings
			a[2] += accADC_BD[2];             // Sum up 512 readings

      if(calibratingA >= MPU_CALI_COUNT_ACC*2-1){
        accZero_BD[0] = a[0] / MPU_CALI_COUNT_ACC;
        accZero_BD[1] = a[1] / MPU_CALI_COUNT_ACC;
        accZero_BD[2] = a[2] / MPU_CALI_COUNT_ACC;
        // test by nishi 2025.3.25
        //accZero[0]=accZero[1]=accZero[2]=0;

        accZeroSum_BD=accZero_BD[0]+accZero_BD[1]+accZero_BD[2];

        // 此処で、acc の内積を出す。
        //accIMZero = sqrt(accZero[0] * accZero[0] + accZero[1] * accZero[1] + accZero[2] * accZero[2]);
        //accZero[YAW] -= ACC_1G;
        //accZero[YAW] = 0;   // 注) これをすると、海抜からの標高値になる。しなければ、起動地点の標高が原点となる。

        calibratingA_f=1;
        calibratingA=0;

        //SERIAL_PORT.println("cBNO086::acc_get_adc(): #3");
        //while(1)
        //  delay(100);

      }
    }
  }

  //accADC[0] =  accRAW[0] - accZero[0];  // [uT]
  //accADC[1] =  accRAW[1] - accZero[1];  // [uT]
  //accADC[2] =  accRAW[2] - accZero[2];  // [uT]

}

void cBNO086::mag_init( void )
{
  uint8_t i;
  for( i=0; i<3; i++ )
  {
    magADC[i]   = 0;
		magZero[i]  = 0;
    magRAW[i]   = 0;
    magADC_BD[i]   = 0.0;
		magZero_BD[i]  = 0.0;
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
bool cBNO086::mag_get_adc( void )
{
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
  byte accuracy;

  bool rc=false;

  //uint8_t data[8];

  //magADC[0]=magADC[1]=magADC[2]=0.0f;

    // is it the correct sensor data we want?
  if (myIMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {

    //magADC_BD[0] = magADC_BD[1] = magADC_BD[2] = 0.0;

    x = myIMU.getMagX();
    y = myIMU.getMagY();
    z = myIMU.getMagZ();
    accuracy = myIMU.getMagAccuracy();

    if(accuracy == 0 || (x==0.0 && y==0.0 && z==0.0)){
      return false;
    }

    magADC_BD[0] = x;  // +_4900
  	magADC_BD[1] = y;  // +_4900
  	magADC_BD[2] = z;  // +_4900

  	
    //magRAW[0] = (data[2] << 8) | data[1];
  	//magRAW[1] = (data[4] << 8) | data[3];
  	//magRAW[2] = (data[6] << 8) | data[5];

  	//if (data[7] & MPU9250_AK8963_OVERFLOW)
  	//if (myICM.agmt.magStat2 & 0x80)
    //{
    //  //SERIAL_PORT.println("mag_get_adc() : #3 overflow");
  	//	return false;
    //}

    //magRAW[0] =magRAW_BD[0] = myICM.agmt.mag.axes.x;  // +_4900
  	//magRAW[1] =magRAW_BD[1] = myICM.agmt.mag.axes.y;  // +_4900
  	//magRAW[2] =magRAW_BD[2] = myICM.agmt.mag.axes.z;  // +_4900

    //#define USE_MAG_DEBUG_X2
    #if defined(USE_MAG_DEBUG_X2)
      SERIAL_PORT.print(F("magRAW: "));
      SERIAL_PORT.print(magADC_BD[0], 3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(magADC_BD[1], 3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(magADC_BD[2], 3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(accuracy);
    #endif

  	mag_common();
    rc=true;
  }
  return true;
}

void cBNO086::mag_common()
{
	static double m[3];  // 4900.0 / 0.15 = 32666.6  , 32666.6*100 = 32,66,666.6

  if(calibratingM_f == 0){
    calibratingM++;
    if(calibratingM >= MPU_CALI_COUNT_MAG){

      if(calibratingM == MPU_CALI_COUNT_ACC){
        m[0]=0.0;
        m[1]=0.0;
        m[2]=0.0;
      }
			m[0] += magADC_BD[0];   // Sum up 512 readings
			m[1] += magADC_BD[1];   // Sum up 512 readings
			m[2] += magADC_BD[2];   // Sum up 512 readings

      if(calibratingM >= MPU_CALI_COUNT_MAG*2-1){
        magZero_BD[0] = m[0] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
        magZero_BD[1] = m[1] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
        magZero_BD[2] = m[2] / MPU_CALI_COUNT_MAG;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant

        // test by nishi 2025.3.25
        //magZero[0]=magZero[1]=magZero[2]=0;

        calibratingM_f=1;
        calibratingM=0;
      }
    }
  }

  //magADC_BD[0] =  magADC_BD[0] - magZero_BD[0];  // [uT]
  //magADC_BD[1] =  magADC_BD[1] - magZero_BD[1];  // [uT]
  //magADC_BD[2] =  magADC_BD[2] - magZero_BD[2];  // [uT]

  // changed by nishi 2025.3.25
  //magADC[0] =  (float)((magRAW[0]>>3) - magZero[0]);
  //magADC[1] =  (float)((magRAW[1]>>3) - magZero[1]);  // [uT]
  //magADC[2] =  (float)((magRAW[2]>>3) - magZero[2]);  // [uT]

}

#if defined(USE_DMP_NISHI)

void cBNO086::dmp_init( void )
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

bool cBNO086::dmp_cali_get_done()
{
	if( calibratingD_f == 1 ) return true;
	else                    return false;
}


bool cBNO086::dmp_get_adc(){

  static double d[4];

  static byte f=0;
  static double q[4]={1.0, 0.0, 0.0, 0.0};

  //#define TRACE_DMP1
  #if defined(TRACE_DMP1)
    SERIAL_PORT.println("dmp_get_adc() start");
  #endif

  bool rc=false;

  // is it the correct sensor data we want?
  if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

    //float quatI = myIMU.getQuatI();
    //float quatJ = myIMU.getQuatJ();
    //float quatK = myIMU.getQuatK();
    //float quatReal = myIMU.getQuatReal();


    float q0 = myIMU.getQuatReal();
    float q1 = myIMU.getQuatI();
    float q2 = myIMU.getQuatJ();
    float q3 = myIMU.getQuatK();
    uint8_t Accuracy = myIMU.getQuatAccuracy();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    //q0=1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
    //if(q0 >= 0.0) q0 = sqrt(q0);
    //else q0 = sqrt(q0 * -1.0) * -1.0;

    #define QUAT_NORM
    #if defined(QUAT_NORM)
    float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0/norm;
    q1 = q1/norm;
    q2 = q2/norm;
    q3 = q3/norm;
    #endif

    //#define TRACE_DMP1A
    #if defined(TRACE_DMP1A)
      //if(quatRadianAccuracy>0.0 && quatRadianAccuracy < quatRadianAccuracy_min){
      //  quatRadianAccuracy_min=quatRadianAccuracy;
      //}
      //SERIAL_PORT.print(dqw, 2);
      //SERIAL_PORT.print(F(","));
      //SERIAL_PORT.print(dqx, 2);
      //SERIAL_PORT.print(F(","));
      //SERIAL_PORT.print(dqy, 2);
      //SERIAL_PORT.print(F(","));
      //SERIAL_PORT.print(dqz, 2);
      //SERIAL_PORT.print(F(","));
      SERIAL_PORT.print(F("Acc: "));
      SERIAL_PORT.print(Accuracy);
      SERIAL_PORT.print(F(" ,RadianAccuracy: "));
      SERIAL_PORT.print(quatRadianAccuracy);
      SERIAL_PORT.println();
    #endif


    //if(!isnan(q0) && Accuracy > 0 && quatRadianAccuracy > 0.04){
    if(!isnan(q0) && quatRadianAccuracy > 0.04){
    //if(!isnan(q0)){
      quatRAW[0] = q0;    // changed by nishi 2025.3.14
      quatRAW[1] = q1;    // changed by nishi 2025.3.14
      quatRAW[2] = q2;    // changed by nishi 2025.3.14
      quatRAW[3] = q3;    // changed by nishi 2025.3.14

      //#define TRACE_DMP1B
      #if defined(TRACE_DMP1B)
        SERIAL_PORT.print(F("check OK"));
        SERIAL_PORT.println();
      #endif


      // キャリブレーション中です。
      if(calibratingD_f == 0){
        calibratingD++;
        if(calibratingD >= MPU_CALI_COUNT_DMP_PRE){
          if(calibratingD == MPU_CALI_COUNT_DMP_PRE){
            d[1]=0;
            d[2]=0;
            d[3]=0;
          }
          d[1] += quatRAW[1];             // Sum up 512 readings
          d[2] += quatRAW[2];             // Sum up 512 readings
          d[3] += quatRAW[3];             // Sum up 512 readings

          // キャリブレーションの終了回数に到達した。
          if(calibratingD >= MPU_CALI_COUNT_DMP+MPU_CALI_COUNT_DMP_PRE-1){
            // DMP の初期誤差を計算する。
            // Z軸のみ補正します。by nishi 2025.3.17
            #define USE_DMP_ADJUST_Z_ONLY
            #if defined(USE_DMP_ADJUST_Z_ONLY)
              quatZero[1] = 0.0;
              quatZero[2] = 0.0;
            #else
              quatZero[1] = d[1] / MPU_CALI_COUNT_DMP;
              quatZero[2] = d[2] / MPU_CALI_COUNT_DMP;
            #endif
            quatZero[3] = d[3] / MPU_CALI_COUNT_DMP; 

            calibratingD_f=1;
            calibratingD = 0;
          }
        }
        // キャリブレーションが終了した直後です。
        if (calibratingD_f == 1)
        {
          // DMP の誤差 quat の実数部を求める。
          quatZero[0] = sqrt(1.0 - ((quatZero[1] * quatZero[1]) + (quatZero[2] * quatZero[2]) + (quatZero[3] * quatZero[3])));    //  -- W    こいつが、バグとの事。

          //quatZero[0] =1.0 - ((quatZero[1] * quatZero[1]) + (quatZero[2] * quatZero[2]) + (quatZero[3] * quatZero[3]));
          //if(quatZero[0] >= 0.0) quatZero[0] = sqrt(quatZero[0]);
          //else quatZero[0] = sqrt(quatZero[0] * -1.0) * -1.0;

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
        #define QUAT_CALIB
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

      //#define TRACE_DMP1B
      #if defined(TRACE_DMP1B)
        //SERIAL_PORT.print(dqw, 2);
        //SERIAL_PORT.print(F(","));
        //SERIAL_PORT.print(dqx, 2);
        //SERIAL_PORT.print(F(","));
        //SERIAL_PORT.print(dqy, 2);
        //SERIAL_PORT.print(F(","));
        //SERIAL_PORT.print(dqz, 2);
        //SERIAL_PORT.print(F(","));
        SERIAL_PORT.print(F("Acc: "));
        SERIAL_PORT.print(quatRadianAccuracy, 2);
        SERIAL_PORT.println();
      #endif

      //quat[0] = (double)dqw;
      //quat[1] = (double)dqx;
      //quat[2] = (double)dqy;
      //quat[3] = (double)dqz;
      rc=true;
    }
  }
  return rc;
}

#endif  


#if defined(USE_AG_NISHI)
/*---------------
bool agm_get_adc()

  get acc, gyro and mag-calibed.
-----------------*/
bool cBNO086::agm_get_adc(){

  //#define TEST_agm_get_adc_1
  #if defined(TEST_agm_get_adc_1)
    SERIAL_PORT.println("agm_get_adc() start");
  #endif

  bool rc=false;
  bool chk_lst[3];
  chk_lst[0]=false;
  chk_lst[1]=false;
  chk_lst[2]=false;

  for(int i=0;i<4;i++){
    chk_lst[0] |= gyro_get_adc();
    chk_lst[1] |= acc_get_adc();
    bool chk;
    chk =chk_lst[0] & chk_lst[1];
    if(chk==true){
      #if defined(TEST_agm_get_adc_1)
        SERIAL_PORT.println("agm_get_adc():#2");
      #endif
      break;
    }
    //delay(10);
    delay(5);   // OK
    //delay(4); // OK or NG
    if(update() == false){
      #if defined(TEST_agm_get_adc_1)
        SERIAL_PORT.println("agm_get_adc():#3");
      #endif
      break;
    }
  }
  rc=chk_lst[0] | chk_lst[1];

  //#define TEST_agm_get_adc_2
  #if defined(TEST_agm_get_adc_2)
    SERIAL_PORT.print("agm_get_adc(): #99 rc=");
    SERIAL_PORT.println(rc);
  #endif

  return rc;
}

void cBNO086::agm_common(){
}
#endif

void cBNO086::acc_cali_start()
{
	//calibratingA = MPU_CALI_COUNT;
  calibratingA_f = 0;
  calibratingA = 0; 
}

bool cBNO086::acc_cali_get_done()
{
	if( calibratingA_f !=0 ) return true;
	else                    return false;
}

bool cBNO086::gyro_cali_get_done()
{
	if( calibratingG_f != 0 ) return true;
	else                    return false;
}

bool cBNO086::mag_cali_get_done()
{
	if( calibratingM_f != 0 ) return true;
	else                    return false;
}

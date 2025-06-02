//----------------------------------------------------------------------------
//    프로그램명 	:
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.ino
//----------------------------------------------------------------------------
#include <Arduino.h>
#include "IMU.h"

#if defined(BOARD_ESP32)
  //#define LED_BUILTIN 17
  #define LED_BUILTIN 4
#elif defined(BOARD_PICO32)
  #define LED_BUILTIN 27
#endif


/*---------------------------------------------------------------------------
     TITLE   : BLE
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cIMU::cIMU(){
  uint8_t i;
  for( i=0; i<3; i++ ){
    rpy[i] = 0.;
  }
	bConnected = false;
  // add by nishi 2025.3.26
  //foxbot3::euler_to_quat<double>(mad_drift_off[0], mad_drift_off[1], mad_drift_off[2], mad_drift_adjust_q);

  #if defined(USE_MADWICK) || defined(USE_MADWICK_2)
    filter.beta=MADWICK_beta;
  #elif defined(USE_MAHONY)
    filter.twoKi=MAHONY_twoKi;
  #endif

}

/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint8_t cIMU::begin( uint32_t hz ){
	uint8_t err_code = IMU_OK;
  uint32_t i;
  uint32_t pre_time;

  #if defined(USE_TRACE)
    SERIAL_PORT.println("cIMU::begin(): #1");
  #endif

  update_hz = hz;
  //update_us = 1000000/hz;
  //update_us = 1000000UL/hz;

  update_us = 1000000UL/ SEN.begin_update_rate;

  v_acc[0]=0.0;
  v_acc[1]=0.0;
  v_acc[2]=0.0;

  v_acc_z[0]=0;
  v_acc_z[1]=0;
  v_acc_z[2]=0;

  tf_dlt[0]=0.0;
  tf_dlt[1]=0.0;
  tf_dlt[2]=0.0;

  quat[0]=1.0;
  quat[1]=0.0;
  quat[2]=0.0;
  quat[3]=0.0;

  quat_tmp_prev[0]=1.0;
  quat_tmp_prev[1]=0.0;
  quat_tmp_prev[2]=0.0;
  quat_tmp_prev[3]=0.0;

  cali_tf=0;    // Madgwick Caliburation
  calibratingMad_f=false; // Madgwick Caliburation
  calibratingMad=0;


  #if defined(BOARD_ESP32)
   	//digitalWrite(LED_BUILTIN, LOW);		// light OFF
    digitalWrite(LED_BUILTIN, HIGH);		// light ON
  #endif

  for(i=0;i<4;i++){
    bConnected = SEN.begin();
    if(bConnected == true)
      break;
    delayMicroseconds(100);        // 100us停止
  }
  
  if( bConnected == true ){
    #if defined(BOARD_ESP32)
      digitalWrite(LED_BUILTIN, LOW);		// light OFF
    #endif

    #if defined(USE_DUAL_FILTERS)
      filter.begin();
      filter2.begin();
    #elif defined(USE_MADWICK) || defined(USE_MADWICK_2)
      filter.begin();
    #endif

    pre_time = millis();

    while(1){
      //SERIAL_PORT.println("cIMU::begin():#4");
      bool c_down=true;
      #if defined(USE_AGM_NISHI)
        c_down &= SEN.acc_cali_get_done();
        c_down &= SEN.gyro_cali_get_done();
        c_down &= SEN.mag_cali_get_done();
      #elif defined(USE_AG_NISHI)
        c_down &= SEN.acc_cali_get_done();
        c_down &= SEN.gyro_cali_get_done();
      #elif defined(USE_DMP_NISHI)
        c_down &= SEN.dmp_cali_get_done();
      #endif
      #if defined(USE_ACC_NISHI)
        c_down &= SEN.acc_cali_get_done();
      #endif
      #if defined(USE_GRYO_NISHI)
        c_down &= SEN.gyro_cali_get_done();
      #endif
      #if defined(USE_MAG)
        c_down &= SEN.mag_cali_get_done();
      #endif
      if(c_down==true)
        break;
      update();
      //if (millis()-pre_time > 25000){
      //  break;
      //}
      //delay(25);        // delay mill sec 1000 [ms]
      delayMicroseconds(update_us);  // micro sec
    }
  }
  else{
    #if defined(BOARD_ESP32)
    	digitalWrite(LED_BUILTIN, HIGH);		// light ON
    #endif
    err_code=1;
  }
	return err_code;
}

/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cIMU::update( uint32_t option ){
  // changed by nishi
  //UNUSED(option);
  bool rc;

  rc=SEN.update();
  if(rc ==false){
    return rc;
  }

  // changed by nishi 2025.3.7
  return computeIMU();
}

#define FILTER_NUM    3

/*---------------------------------------------------------------------------
     TITLE   : computeIMU
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cIMU::computeIMU( void ){
  static unsigned long prev_process_time = micros();
  static unsigned long cur_process_time = 0;
  static unsigned long process_time = 0;
  uint32_t i;
  static int32_t gyroADC[3][FILTER_NUM] = {0,};
  int32_t gyroAdcSum;

  static int32_t accADC[3][FILTER_NUM] = {0,};
  int32_t accAdcSum;

  uint32_t axis;

  bool rc=true;   // add by nishi 2025.3.7

  unsigned long cur_ms = millis();

  // add by nishi 2025.3.26
  static double d[4];

  #if defined(USE_ACC_NISHI)
    // Get Acc data
	  SEN.acc_get_adc();
  #endif
  #if defined(USE_GRYO_NISHI)
    // Get Gyro data
	  SEN.gyro_get_adc();
  #endif

  #if defined(USE_MAG)
    // Get Magnet data
    if(SEN.mag_get_adc() == false){
      //return false;
    }
  #endif

  #if defined(USE_DMP_NISHI)
    // Get DMP data
    if(SEN.dmp_get_adc()!=true){
      //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
      return false;
    }
    // DMP Caliburation not yet?
    if(SEN.calibratingD_f == 0){
      return false;
    }
    // use ICM20948 DMP Fusion.  add by nishi 2021.11.6
    quat[0] = SEN.quat[0];  // W
    quat[1] = SEN.quat[1];  // X
    quat[2] = SEN.quat[2];  // Y
    quat[3] = SEN.quat[3];  // Z
    return true;
  #endif
  #if defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
    if(SEN.agm_get_adc()!=true){
      //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the blue
      return false;
    }
  #endif

  //#define TEST_computeIMU_1
  #if defined(TEST_computeIMU_1)
    SERIAL_PORT.println(F("cIMU::computeIMU(): #2"));
  #endif

  #if defined(USE_ACC_NISHI) || defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
    // Acc Caliburation not yet?
    if(SEN.calibratingA_f == 0){
      return false;
    }
  #endif

  //SERIAL_PORT.println(F("cIMU::computeIMU(): #2.1"));

  #if defined(USE_GRYO_NISHI) || defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
    if(SEN.calibratingG_f == 0){
      return false;
    }
  #endif

  //SERIAL_PORT.println(F("cIMU::computeIMU(): #2.2"));

  #if defined(USE_MAG) || defined(USE_AGM_NISHI)
    if(SEN.calibratingM_f == 0){
      return false;
    }
  #endif

  //#define TEST_computeIMU_2
  #if defined(TEST_computeIMU_2)
    SERIAL_PORT.println(F("cIMU::computeIMU(): #3"));
  #endif


  #if defined(USE_ACC_NISHI_X)
    // ACC 平滑化
    for (axis = 0; axis < 3; axis++){
      accADC[axis][0] = SEN.accADC[axis];
      accAdcSum = 0;
      for (i=0; i<FILTER_NUM; i++){
        accAdcSum += accADC[axis][i];
      }
      SEN.accADC[axis] = accAdcSum/FILTER_NUM;
      for (i=FILTER_NUM-1; i>0; i--){
        accADC[axis][i] = accADC[axis][i-1];
      }
    }
  #endif

  #if defined(USE_GRYO_NISHI_X)
    // GYRO 平滑化
    for (axis = 0; axis < 3; axis++){
      gyroADC[axis][0] = SEN.gyroADC[axis];
      gyroAdcSum = 0;
      for (i=0; i<FILTER_NUM; i++){
        gyroAdcSum += gyroADC[axis][i];
      }
      SEN.gyroADC[axis] = gyroAdcSum/FILTER_NUM;
      for (i=FILTER_NUM-1; i>0; i--){
        gyroADC[axis][i] = gyroADC[axis][i-1];
      }
      if (abs(SEN.gyroADC[axis]) <= 3){
        SEN.gyroADC[axis] = 0;
      }
    }
  #endif

  for( i=0; i<3; i++ ){
    #if defined(USE_ACC_NISHI) || defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
      accRaw[i]   = SEN.accRAW[i];
      accData[i]  = SEN.accADC[i];
    #endif
    #if defined(USE_GRYO_NISHI) || defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
      gyroRaw[i]  = SEN.gyroRAW[i];
      gyroData[i] = SEN.gyroADC[i];
    #endif
    #if defined(USE_MAG) || defined(USE_AGM_NISHI)
      magRaw[i]   = SEN.magRAW[i];
      magData[i]  = SEN.magADC[i];
    #endif
  }

  // 調整1.
  // こでは、 Acc の生データをチェックします。
  // IMU を 3軸方向に、個別に、5[cm] ほど、動かして停止させる、移動テスト をします。
  // Arduino IDE Serial Plotter で、波形を観測して、
  // 移動-停止毎に、上下の波形(山)の面積が大体同じか、確認します。
  // 上下の波形(山)の面積が違う場合、特に、後の面積が大きいいばあいは、IMU の精度不足です。
  // 別の IMU を使いましょう。
  //#define TEST_NISHI_5_K
  #if defined(TEST_NISHI_5_K)
    //SERIAL_PORT.print(F("SEN.accZero[2]:"));
    //SERIAL_PORT.print(SEN.accZero[2]);
    SERIAL_PORT.print(F("accData[0]:"));
    SERIAL_PORT.print(accData[0]-SEN.accZero[0]);
    SERIAL_PORT.print(F(" accData[1]:"));
    SERIAL_PORT.print(accData[1]-SEN.accZero[1]);
    SERIAL_PORT.print(F(" accData[2]:"));
    SERIAL_PORT.println(accData[2]-SEN.accZero[2]);
  #endif

  #if defined(USE_ACC_NISHI) || defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
    #if defined(BNO086_IMU) || defined(ADAF_BNO086_IMU) || defined(ADAF_BNO055_IMU)
      ax0  = SEN.accADC_BD[0];
      ay0  = SEN.accADC_BD[1];
      az0  = SEN.accADC_BD[2];
    #else
      ax0 = (float)accData[0]*SEN.aRes; // [g]
      ay0 = (float)accData[1]*SEN.aRes; // [g]
      az0 = (float)accData[2]*SEN.aRes; // [g]

      // ノーマライズしてみる
      //float accIMZero = sqrt(ax0 * ax0 + ay0 * ay0 + az0 * az0);
      //ax0 /= accIMZero;
      //ay0 /= accIMZero;
      //az0 /= accIMZero;


      //ax0 = SEN.accADC_BD[0]; // scaled data Acc [mg]
      //ay0 = SEN.accADC_BD[1]; // scaled data Acc [mg]
      //az0 = SEN.accADC_BD[2]; // scaled data Acc [mg]
      #endif

    //#define TEST_NISHI_5_L
    #if defined(TEST_NISHI_5_L)
      SERIAL_PORT.print(F("accIMU:"));
      SERIAL_PORT.print(ax0,6);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(ay0,6);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(az0,6);
    #endif
  #endif

  #if defined(USE_GRYO_NISHI) || defined(USE_AG_NISHI) || defined(USE_AGM_NISHI)
    #if defined(BNO086_IMU) || defined(ADAF_BNO086_IMU) || defined(ADAF_BNO055_IMU) || defined(ICM20948_IMU) || defined(ICM42688_IMU)
      gx0 = SEN.gyroADC_BD[0];    // [dps] or [rds]
      gy0 = SEN.gyroADC_BD[1];    // [dps] or [rds]
      gz0 = SEN.gyroADC_BD[2];    // [dps] or [rds]
    #else
      gx0 = SEN.gyroADC[0]*SEN.gRes;    // data [dps]
      gy0 = SEN.gyroADC[1]*SEN.gRes;    // [dps]
      gz0 = SEN.gyroADC[2]*SEN.gRes;    // [dps]

      //gx0 = SEN.gyroADC_BD[0];    // scaled data [dps]
      //gy0 = SEN.gyroADC_BD[1];    // [dps]
      //gz0 = SEN.gyroADC_BD[2];    // [dps]
    #endif

    // add by nishi 2025.4.10
    // Convert gyroscope degrees/sec to radians/sec
    #if !defined(ADAF_BNO086_IMU) && !defined(ADAF_BNO055_IMU)
      gx0 *= 0.0174533f;
      gy0 *= 0.0174533f;
      gz0 *= 0.0174533f;
    #endif

    //#define TEST_NISHI_5_O
    #if defined(TEST_NISHI_5_O)
      SERIAL_PORT.print(F("gyroIMU:"));
      SERIAL_PORT.print(gx0);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(gy0);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(gz0);
    #endif
  #endif

  #if defined(USE_MAG) || defined(USE_AGM_NISHI)
    #if defined(BNO086_IMU) || defined(ADAF_BNO086_IMU) || defined(ICM20948_COMBI_IMU)
      // Apply mag offset compensation (base values in uTesla)
      //float x = SEN.magADC[0] - mag_offsets[0];
      //float y = SEN.magADC[1] - mag_offsets[1];
      //float z = SEN.magADC[2] - mag_offsets[2];

      // Apply mag soft iron error compensation
      //mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
      //my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
      //mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

      mx0 = SEN.magADC_BD[0];  // +_4900 * 0.15 = +_735.0[uT]
      my0 = SEN.magADC_BD[1];  // +_4900 * 0.15 = +_735.0[uT]
      mz0 = SEN.magADC_BD[2];  // +_4900 * 0.15 = +_735.0[uT]
      //
      // +_735/180.0 = +_4.083333333333333

      //#define TEST_MAG_ONLY_TF1
      #if defined(TEST_MAG_ONLY_TF1)
        if(mx0==0.0 && my0==0.0 && mz0==0.0)
          return false;
        // Magnetmeter だけで、pose を出してみる。
        // atan2(x,y) -> [rad]
        float x=mx0;
        float y=my0;
        float heading_m = -1.0 * atan2(y,x); // [rad]
        heading_m = foxbot3::normalize_tf_rz(heading_m);  // [rad]
        foxbot3::euler_to_quat<float>(0,0,heading_m,quat);

        return true;
      #endif
    #elif defined(ICM20948_IMU)
      mx0 = SEN.mx0;
      my0 = SEN.my0;
      mz0 = SEN.mz0;

      //#define TEST_MAG_ONLY_TF0
      #if defined(TEST_MAG_ONLY_TF0)
        // Magnetmeter だけで、pose を出してみる。
        // heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;
        // atan2(x,y) -> [rad]
        float heading_m = atan2(my0,mx0); // [rad]

        //SERIAL_PORT.print(F("heading_m:"));
        //SERIAL_PORT.println(heading_m);

        //heading_m = foxbot3::normalize_tf_rz(heading_m);  // [rad]
        foxbot3::euler_to_quat<float>(0,0,heading_m,quat);

        return true;
      #endif


    #else
      //float x = (float)SEN.magADC[0]*SEN.mRes/1000000.0;  // scaled data [uT] -> [T]
      //float y = (float)SEN.magADC[1]*SEN.mRes/1000000.0;  // scaled data [uT] -> [T]
      //float z = (float)SEN.magADC[2]*SEN.mRes/1000000.0;  // scaled data [uT] -> [T]

      //float x = (float)SEN.magADC[0]*SEN.mRes;  // scaled data [uT]
      //float y = (float)SEN.magADC[1]*SEN.mRes;  // scaled data [uT]
      //float z = (float)SEN.magADC[2]*SEN.mRes;  // scaled data [uT]

      //float x = SEN.magADC_BD[0];
      //float y = SEN.magADC_BD[1];
      //float z = SEN.magADC_BD[2];

      //#define TEST_MAG_ONLY_TF
      #if defined(TEST_MAG_ONLY_TF)
        // Magnetmeter だけで、pose を出してみる。
        // heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;
        // atan2(x,y) -> [rad]
        float heading_m = atan2(y,x); // [rad]
        heading_m = foxbot3::normalize_tf_rz(heading_m);  // [rad]
        foxbot3::euler_to_quat<float>(0,0,heading_m,quat);

        return true;
      #endif
      mx0=0.0;
      my0=0.0;
      mz0=0.0;
      if(SEN.magADC[0] != 0 && SEN.magADC[1] != 0 && SEN.magADC[2] != 0){

        //float x = (float)SEN.magADC[0]*SEN.mRes - mag_offsets[0];  // scaled data [uT]
        //float y = (float)SEN.magADC[1]*SEN.mRes - mag_offsets[1];  // scaled data [uT]
        //float z = (float)SEN.magADC[2]*SEN.mRes - mag_offsets[2];  // scaled data [uT]

        float x = (float)SEN.magADC[0] - SEN.mag_offsets[0];  // magneto raw data
        float y = (float)SEN.magADC[1] - SEN.mag_offsets[1];  // magneto raw data
        float z = (float)SEN.magADC[2] - SEN.mag_offsets[2];  // magneto raw data

        // Apply mag soft iron error compensation
        mx0 = x * SEN.mag_softiron_matrix[0][0] + y * SEN.mag_softiron_matrix[0][1] + z * SEN.mag_softiron_matrix[0][2];
        my0 = x * SEN.mag_softiron_matrix[1][0] + y * SEN.mag_softiron_matrix[1][1] + z * SEN.mag_softiron_matrix[1][2];
        mz0 = x * SEN.mag_softiron_matrix[2][0] + y * SEN.mag_softiron_matrix[2][1] + z * SEN.mag_softiron_matrix[2][2];

        //#define TEST_MAG_ONLY_TF3
        #if defined(TEST_MAG_ONLY_TF3)
          float Mxyz[3];
          Mxyz[0]=mx0;
          Mxyz[1]=my0;
          Mxyz[2]=mz0;
          vector_normalize(Mxyz);
          // Magnetmeter だけで、pose を出してみる。
          // atan2(x,y) -> [rad]
          //float heading_m = -1.0 * atan2(my0,mx0); // [rad]
          float heading_m = -1.0 * atan2(Mxyz[0],Mxyz[1]); // [rad]
          //heading_m = foxbot3::normalize_tf_rz(heading_m);  // [rad]
          //foxbot3::euler_to_quat<float>(0,0,heading_m,quat);
          heading_m = heading_m * 180.0 / M_PI; // rad to degree
          SERIAL_PORT.print(F("heading_m:"));
          SERIAL_PORT.println(heading_m);
    
          return true;
        #endif

        #if defined(USE_MAHONY)
          // Mahony
          // my、mz は、極性を逆にする。
          //mx0 = SEN.magADC_BD[0];
          my0 = my0 * -1.0;
          mz0 = mz0 * -1.0;
        #elif defined(USE_MADWICK) || defined(USE_MADWICK2)
          // madgwick
          // my は、極性を逆にする。
          my0 = my0 * -1.0;
        #endif

        // test
        //#define USE_MAG_NORM1
        #if defined(USE_MAG_NORM1)
          float temp[3];

          //Mag scale divide by 369.4 to normalize
          float M_B[3] = { -156.70,  -52.79, -141.07};

          float M_Ainv[3][3]
          { {  1.12823, -0.01142,  0.00980},
            { -0.01142,  1.09539,  0.00927},
            {  0.00980,  0.00927,  1.10625}
          };

          // local magnetic declination in degrees
          float declination = -14.84;
          float Mxyz[3];
          Mxyz[0]=(float)SEN.magADC[0];
          Mxyz[1]=(float)SEN.magADC[1];
          Mxyz[2]=(float)SEN.magADC[2];

          for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
          Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
          Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
          Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
          vector_normalize(Mxyz);

          // my、mz は、極性を逆にする。
          //mx0 = x;
          //my0 = y * -1.0;
          //mz0 = z * -1.0;

          mx0 = Mxyz[0];
          my0 = Mxyz[1] * -1.0;
          mz0 = Mxyz[2] * -1.0;
        #endif
      }
    #endif

    //#define TEST_NISHI_5_2
    #if defined(TEST_NISHI_5_2)
      float mxu = (float)SEN.magADC[0]*SEN.mRes;  // scaled data [uT]
      float myu = (float)SEN.magADC[1]*SEN.mRes;  // scaled data [uT]
      float mzu = (float)SEN.magADC[2]*SEN.mRes;  // scaled data [uT]
      SERIAL_PORT.print(F("magIMU:"));
      SERIAL_PORT.print(mxu,3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(myu,3);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(mzu,3);
      // magIMU:97.500 ,29.250 ,40.800 -->  1桁大きいので  * 0.1 すべきか!!
    #endif

  #elif defined(USE_AGM_NISHI)
    #if defined(ADAF_BNO086_IMU)
      mx0 = SEN.magADC_BD[0];  // +_4900 * 0.15 = +_735.0[uT]
      my0 = SEN.magADC_BD[1];  // +_4900 * 0.15 = +_735.0[uT]
      mz0 = SEN.magADC_BD[2];  // +_4900 * 0.15 = +_735.0[uT]
    #else
      float x = (float)SEN.magADC32[0]*SEN.mRes;  // scaled data [uT]
      float y = (float)SEN.magADC32[1]*SEN.mRes;  // scaled data [uT]
      float z = (float)SEN.magADC32[2]*SEN.mRes;  // scaled data [uT]

      //float x = SEN.magADC_BD[0];
      //float y = SEN.magADC_BD[1];
      //float z = SEN.magADC_BD[2];

      //#define TEST_MAG_ONLY_TF2
      #if defined(TEST_MAG_ONLY_TF2)
        // Magnetmeter だけで、pose を出してみる。
        // atan2(x,y) -> [rad]
        float heading_m = atan2(y,x); // [rad]
        heading_m = foxbot3::normalize_tf_rz(heading_m);  // [rad]
        foxbot3::euler_to_quat<float>(0,0,heading_m,quat);

        return true;
      #endif

      mx0 = x;
      //my0 = y * -1.0;
      my0 = y;
      //mz0 = z * -1.0;
      mz0 = z;
    #endif

  #endif

  //#define TEST_computeIMU_3
  #if defined(TEST_computeIMU_3)
    SERIAL_PORT.println(F("cIMU::computeIMU():#5"));
  #endif

  cur_process_time  = micros();
  process_time      = cur_process_time-prev_process_time;
  prev_process_time = cur_process_time;

  #if defined(USE_MADWICK) || defined(USE_MADWICK_2) || defined(USE_MAHONY) || defined(USE_DUAL_FILTERS)
    #if defined(USE_DUAL_FILTERS)
      filter.invSampleFreq = (float)process_time/1000000.0f;
      #if defined(IMU_SENSER6)
        filter.MadgwickAHRSupdateIMU(gx0, gy0, gz0, ax0, ay0, az0);
      #else
        filter.MadgwickAHRSupdate(gx0, gy0, gz0, ax0, ay0, az0, mx0, my0, mz0);
      #endif
      quat_tmp[0] = filter.q0;  // W
      quat_tmp[1] = filter.q1;  // X
      quat_tmp[2] = filter.q2;  // Y
      quat_tmp[3] = filter.q3;  // Z

      #if defined(IMU_SENSER6)
        filter2.MahonyAHRSupdateIMU(gx0, gy0, gz0, ax0, ay0, az0);
        //filter.MadgwickAHRSupdateIMU(gx0, gy0, gz0, ax0, ay0, az0);
      #else
        filter2.MahonyAHRSupdate(gx0, gy0, gz0, ax0, ay0, az0, mx0, my0, mz0);
        //filter.MadgwickAHRSupdate(gx0, gy0, gz0, ax0, ay0, az0, mx0, my0, mz0);
      #endif
      quat2_tmp[0] = filter2.q0;  // W
      quat2_tmp[1] = filter2.q1;  // X
      quat2_tmp[2] = filter2.q2;  // Y
      quat2_tmp[3] = filter2.q3;  // Z

    #elif defined(USE_MADWICK)
      filter.invSampleFreq = (float)process_time/1000000.0f;
      #if defined(IMU_SENSER6)
        filter.MadgwickAHRSupdateIMU(gx0, gy0, gz0, ax0, ay0, az0);
      #else
        filter.MadgwickAHRSupdate(gx0, gy0, gz0, ax0, ay0, az0, mx0, my0, mz0);
      #endif
      quat_tmp[0] = filter.q0;  // W
      quat_tmp[1] = filter.q1;  // X
      quat_tmp[2] = filter.q2;  // Y
      quat_tmp[3] = filter.q3;  // Z

      //rpy[0] = filter.getRoll();
      //rpy[1] = filter.getPitch();
      //rpy[2] = filter.getYaw()-180.;

      //angle[0] = (int16_t)(rpy[0] * 10.);
      //angle[1] = (int16_t)(rpy[1] * 10.);
      //angle[2] = (int16_t)(rpy[1] * 1.);
    #elif defined(USE_MAHONY)
      filter.invSampleFreq = (float)process_time/1000000.0f;
      #if defined(IMU_SENSER6)
        filter.MahonyAHRSupdateIMU(gx0, gy0, gz0, ax0, ay0, az0);
        //filter.MadgwickAHRSupdateIMU(gx0, gy0, gz0, ax0, ay0, az0);
      #else
        filter.MahonyAHRSupdate(gx0, gy0, gz0, ax0, ay0, az0, mx0, my0, mz0);
        //filter.MadgwickAHRSupdate(gx0, gy0, gz0, ax0, ay0, az0, mx0, my0, mz0);
      #endif
      quat_tmp[0] = filter.q0;  // W
      quat_tmp[1] = filter.q1;  // X
      quat_tmp[2] = filter.q2;  // Y
      quat_tmp[3] = filter.q3;  // Z

    #elif defined(USE_MADWICK_2)
      float delta = (float)process_time/1000000.0f;
      filter.MadgwickQuaternionUpdate(ax0, ay0, az0, gx0, gy0, gz0,mx0, my0, mz0, delta);

      //filter.getQuaternion(&quat[0], &quat[1], &quat[2], &quat[3]);
      quat_tmp[0] = filter.q[0];  // W
      quat_tmp[1] = filter.q[1];  // X
      quat_tmp[2] = filter.q[2];  // Y
      quat_tmp[3] = filter.q[3];  // Z
    #endif


    //#define TEST_NISHI_5_D
    #if defined(TEST_NISHI_5_D)
      SERIAL_PORT.print(F("quat_tmp:"));
      SERIAL_PORT.print(quat_tmp[0], 8);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(quat_tmp[1], 8);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.print(quat_tmp[2], 8);
      SERIAL_PORT.print(F(" ,"));
      SERIAL_PORT.println(quat_tmp[3], 8);
    #endif

  #endif


  #if defined(USE_IMU_DIST)
    // Cumpute CB 
    //compCB(quat_tmp,&cb);

    // 1つ前のとの 4:6 のクォータニオンをつかいます。
    quat_tmp_prev[1] = (quat_tmp[1]*0.3 + quat_tmp_prev[1]*0.7);
    quat_tmp_prev[2] = (quat_tmp[2]*0.3 + quat_tmp_prev[2]*0.7);
    quat_tmp_prev[3] = (quat_tmp[3]*0.3 + quat_tmp_prev[3]*0.7);

    //q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));    //  -- W
    quat_tmp_prev[0] = sqrt(1.0 - ((quat_tmp_prev[1] * quat_tmp_prev[1]) + (quat_tmp_prev[2] * quat_tmp_prev[2]) + (quat_tmp_prev[3] * quat_tmp_prev[3])));

    compCB(quat_tmp_prev,&cb);

    double acc_Zero[3];

    // acc 1G の分配値を計算
    //acc_Zero[0] = cb.dt[2][0]*(ACC_1G - SEN.zero_off);  
    //acc_Zero[1] = cb.dt[2][1]*(ACC_1G - SEN.zero_off);  
    //acc_Zero[2] = cb.dt[2][2]*(ACC_1G - SEN.zero_off);
    acc_Zero[0] = cb.dt[2][0]*((double)SEN.accZero[2] - SEN.zero_off);  
    acc_Zero[1] = cb.dt[2][1]*((double)SEN.accZero[2] - SEN.zero_off);  
    acc_Zero[2] = cb.dt[2][2]*((double)SEN.accZero[2] - SEN.zero_off);


    // acc 計測値から、1G をキャンセルします。 -> 読み込み値での計算
    // SEN.zero_off(ACC_ZERO_OFF) を調整して、az が、 0軸 近辺を 中心 に振幅するようにします。
    ax = (double)accData[0] - acc_Zero[0];
    //ax = (double)accData[0] - acc_Zero[1];
    ay = (double)accData[1] - acc_Zero[1];
    //ay = (double)accData[1] - acc_Zero[0];
    az = (double)accData[2] - acc_Zero[2];

    quat_tmp_prev[0]=quat_tmp[0];
    quat_tmp_prev[1]=quat_tmp[1];
    quat_tmp_prev[2]=quat_tmp[2];
    quat_tmp_prev[3]=quat_tmp[3];

    compCB(quat_tmp,&cb);

    // 調整2.
    // こでは、 Acc の 1G キャンセルのバイアス値をチェックします。
    // IMU を 静止させた状態でテストします。
    // Arduino IDE Serial Plotter で、波形を観測して、
    // Z 軸の波形の中心が、0 近辺になるように、ICM20948.h の ACC_ZERO_OFF を調整します。
    // 此処の調整が、自動化出来れば、ありがたいです。
    // 調整3.
    // IMU を 3軸方向に、個別に、5[cm] ほど、動かして停止させる、移動テスト をします。
    // Arduino IDE Serial Plotter で、波形を観測して、
    // 移動-停止毎に、上下に、半波(1山)が、同じ様に観測されれば、OK です。
    // 同じ様に、観測されなければ、MadgwickAHRS.cpp の betaDef を調整します。
    // 同じ面積にならないと、揺り戻しの、誤った動きをします。
    // こは、大体で、OK です。最終調整は、調整5. で出来ます。
    //#define TEST_11_3
    #if defined(TEST_11_3)
      //if(az > -3000 && fabs(az) < 15000){
      SERIAL_PORT.print(F("ax:"));
      SERIAL_PORT.print(ax,6);
      SERIAL_PORT.print(F(" ay:"));
      SERIAL_PORT.print(ay,6);
      SERIAL_PORT.print(F(" az:"));
      SERIAL_PORT.println(az,6);
      //}
    #endif

    // acc のノイズの 削除
    if (fabs(ax) <= ACC_X_CUT_OFF) ax = 0;
    if (fabs(ay) <= ACC_Y_CUT_OFF) ay = 0;
    if (fabs(az) <= ACC_Z_CUT_OFF_P) az = 0;

    // 加速度の履歴を作成
    v_acc_z[0] <<=1;
    v_acc_z[1] <<=1;
    v_acc_z[2] <<=1;
    if(ax != 0)
      v_acc_z[0] +=1;
    if(ay != 0)
      v_acc_z[1] +=1;
    if(az != 0)
      v_acc_z[2] +=1;
    
    //v_acc_z[0] &= 0x00ff;
    //v_acc_z[1] &= 0x00ff;
    //v_acc_z[2] &= 0x00ff;

    // 調整4.
    // こでは、 Acc の 1G 時の CUT OFF を決めます。
    // IMU を 静止させた状態でテストします。
    // Arduino IDE Serial Plotter で、波形を観測して、
    // ax,ay,az が、0 表示されるか確認します。
    // もし、波形が観測されたなら、ACC_X_CUT_OFF、ACC_Y_CUT_OFF、ACC_Z_CUT_OFF_P を調整します。
    // この CUT_OFF は、なるべく小さいほうが良いです。
    //#define TEST_11_5
    #if defined(TEST_11_5)
      //if(az > -3000 && fabs(az) < 15000){
        SERIAL_PORT.print(F("ax:"));
        SERIAL_PORT.print(ax,6);
        SERIAL_PORT.print(F(" ay:"));
        SERIAL_PORT.print(ay,6);
        SERIAL_PORT.print(F(" az:"));
        SERIAL_PORT.println(az,6);
      //}
    #endif

    // Madgwick Caliburation OK?
    if(cali_tf >= 7000){
      // こちらは、調整3. と同じですが、MadgwickAHRS のキャリブレーションごのチェックになります。
      //#define TEST_11_6
      #if defined(TEST_11_6)
        SERIAL_PORT.print(F("ax:"));
        SERIAL_PORT.print(ax,6);
        SERIAL_PORT.print(F(" ay:"));
        SERIAL_PORT.print(ay,6);
        SERIAL_PORT.print(F(" az:"));
        SERIAL_PORT.println(az,6);
      #endif

      quat[0] = quat_tmp[0];  // W
      quat[1] = quat_tmp[1];  // X
      quat[2] = quat_tmp[2];  // Y
      quat[3] = quat_tmp[3];  // Z

      computeTF(process_time);
    }
    else{
      cali_tf ++;
    }
    
  #elif defined(USE_MADWICK) || defined(USE_MADWICK_2) || defined(USE_MAHONY) || defined(USE_DUAL_FILTERS)
    // キャリブレーション中です。
    if(calibratingMad_f == false){
      //#define TEST_11_7
      #if defined(TEST_11_7)
        SERIAL_PORT.print(F("computrIMU():#6"));
        SERIAL_PORT.println("");
      #endif
      calibratingMad++;
      if(calibratingMad >= IMU_CALI_COUNT_DMP_PRE){
        if(calibratingMad == IMU_CALI_COUNT_DMP_PRE){
          d[1]=d[2]=d[3]=0;
        }
        #if defined(USE_MADWICK) || defined(USE_MAHONY) || defined(USE_DUAL_FILTERS)
          d[1] += filter.q1;  // X
          d[2] += filter.q2;  // y
          d[3] += filter.q3;  // z
        #else
          //d[0] += filter.q[0];  // W
          d[1] += filter.q[1];  // X
          d[2] += filter.q[2];  // Y
          d[3] += filter.q[3];  // Z
        #endif

        // キャリブレーションの終了回数に到達した。
        if(calibratingMad >= IMU_CALI_COUNT_DMP_PRE+IMU_CALI_COUNT_DMP-1){
          // Z軸のみ補正します。by nishi 2025.3.17
          //quatZero[1] = d[1] / IMU_CALI_COUNT_DMP;
          quatZero[1] = 0.0;
          //quatZero[2] = d[2] / IMU_CALI_COUNT_DMP;
          quatZero[2] = 0.0;
          quatZero[3] = d[3] / IMU_CALI_COUNT_DMP; 

          calibratingMad_f = true;
          calibratingMad = 0;

          //#define TEST_11_8
          #if defined(TEST_11_8)
            SERIAL_PORT.print(F("computrIMU():#7"));
            SERIAL_PORT.println("");
          #endif

        }
      }
      // キャリブレーションが終了した直後です。
      if (calibratingMad_f == true)
      {
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
      rc=false;
    }
    // キャリブレーション完了後です。
    else{
      #if defined(USE_DUAL_FILTERS)
        quatRAW2[0] = filter2.q0;  // W
        quatRAW2[1] = filter2.q1;  // X
        quatRAW2[2] = filter2.q2;  // Y
        quatRAW2[3] = filter2.q3;  // Z
      #endif
      #if defined(USE_MADWICK) || defined(USE_MAHONY) || defined(USE_DUAL_FILTERS)
        quatRAW[0] = filter.q0;  // W
        quatRAW[1] = filter.q1;  // X
        quatRAW[2] = filter.q2;  // Y
        quatRAW[3] = filter.q3;  // Z
      #else
        quatRAW[0] = filter.q[0];  // W
        quatRAW[1] = filter.q[1];  // X
        quatRAW[2] = filter.q[2];  // Y
        quatRAW[3] = filter.q[3];  // Z
      #endif

      // 起動時の向きの補正をする。
      #define USE_POSE_ADJUST
      #if defined(USE_POSE_ADJUST)
        // Madgwick の zを補正
        foxbot3::Kakezan(quatZeroK, quatRAW, quat);
        #if defined(USE_DUAL_FILTERS)
          double roll1,pitch1,yaw1;
          double roll2,pitch2,yaw2;
          // get adjusted roll,pitch,yaw
          foxbot3::QuaternionToEulerAngles(quat, roll1, pitch1, yaw1);
          // get Mahony roll,pitch,yaw
          foxbot3::QuaternionToEulerAngles(quatRAW2, roll2, pitch2, yaw2);
          // mixd Mahony x,y and Madgwick adjusted z
          foxbot3::euler_to_quat<double>(roll2, pitch2, yaw1,quat);

          // Mahony の q3 を書き換える。
          // get Madgwick roll,pitch,yaw
          foxbot3::QuaternionToEulerAngles(quatRAW, roll1, pitch1, yaw1);
          foxbot3::euler_to_quat<double>(roll2, pitch2, yaw1,quatRAW2);

          filter2.q0 = quatRAW2[0];
          filter2.q1 = quatRAW2[1];
          filter2.q2 = quatRAW2[2];
          filter2.q3 = quatRAW2[3];

        #endif
      #else
        #if defined(USE_MADWICK) || defined(USE_MAHONY) || defined(USE_DUAL_FILTERS)
          quat[0] = filter.q0;  // W
          quat[1] = filter.q1;  // X
          quat[2] = filter.q2;  // Y
          quat[3] = filter.q3;  // Z
        #else
          quat[0] = filter.q[0];  // W
          quat[1] = filter.q[1];  // X
          quat[2] = filter.q[2];  // Y
          quat[3] = filter.q[3];  // Z
        #endif
      #endif

      //#define TEST_NISHI_6_D
      #if defined(TEST_NISHI_6_D)
        SERIAL_PORT.print(F("quat:"));
        SERIAL_PORT.print(quat[0], 8);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.print(quat[1], 8);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.print(quat[2], 8);
        SERIAL_PORT.print(F(" ,"));
        SERIAL_PORT.println(quat[3], 8);
      #endif

    }
  #endif

  //#define TEST_NISHI_6_E
  #if defined(TEST_NISHI_6_E)
    SERIAL_PORT.print(F("cIMU::comuteIMU rc:"));
    SERIAL_PORT.println(rc);
  #endif

  // add by nishi 2025.3.7
  return rc;

}

#if defined(USE_IMU_DIST)
/*---------------------------------------------------------------------------
     TITLE   : computeTF
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeTF(unsigned long process_time){

  double dist[3];   // ロボット座標系 今回の移動距離 
  double dlt[3];    // 基準座標系 今回の移動距離
  //double v_acc_dlt[3];

  double s = (double)process_time/1000000.0;

  dist[0]=0;
  dist[1]=0;
  dist[2]=0;

  //QuaternionToEulerAngles(quat[0], quat[1], quat[2], quat[3],roll, pitch, yaw);

  // 今のロボット速度を計算。   [ax ay az]*s を積分 
  v_acc[0] += ax*s*SEN.aRes;
  v_acc[1] += ay*s*SEN.aRes;
  v_acc[2] += az*s*SEN.aRes;


  // 調整5.
  // IMU を 3軸方向に、個別に、5[cm] ほど、動かして停止させる、移動テスト をします。
  // Arduino IDE Serial Plotter で、波形を観測して、
  // 移動-停止毎に、半波(1山)が、きれいに観測されれば、OK です。
  // 半波(1山)が、観測されなければ、MadgwickAHRS.cpp の betaDef を調整します。
  #define KKKK2
  #if defined(KKKK2)
    SERIAL_PORT.print(F("v_acc[0]:"));
    SERIAL_PORT.print(v_acc[0]*1000.0,8);
    SERIAL_PORT.print(F(" v_acc[1]:"));
    SERIAL_PORT.print(v_acc[1]*1000.0,8);
    SERIAL_PORT.print(F(" v_acc[2]:"));
    SERIAL_PORT.println(v_acc[2]*1000.0,8);
  #endif

  // v_acc[0]:-1.672498 v_acc[1]:-0.916505 v_acc[2]:0.888893

  // 速度を Cut Off
  //#define VX_CUT_OFF 0.4
  //#define VY_CUT_OFF 0.4
  //#define VZ_CUT_OFF 0.4

  // 加速度=0 の時の暴走の対応
  // 低速域で、一定時間、x速度が常に同じだと、速度をクリアします。
  if(fabsf(v_acc[0]) <= VX_CUT_OFF){
    if ((v_acc_z[0] & 0x00ff) == 0){
      // 通り過ぎた分だけ戻す
      dist[0] -= v_acc[0]*s*7.0;
      v_acc[0]=0.0;
    }
  }
  // 高速域で、一定時間、x速度が常に同じだと、速度をクリアします。
  else if(fabsf(v_acc[0]) <= VX_MAX_CUT_OFF && v_acc_z[0] == 0){
    // 通り過ぎた分だけ戻す
    dist[0] -= v_acc[0]*s*15.0;
    v_acc[0]=0.0;
  }

  // 低速域で、一定時間、y速度が常に同じだと、速度をクリアします。
  if(fabsf(v_acc[1]) <= VY_CUT_OFF){
    if ((v_acc_z[1] & 0x00ff) == 0){
      // 通り過ぎた分だけ戻す
      dist[1] -= v_acc[1]*s*7.0;
      v_acc[1]=0.0;
    }
  }
  // 高速域で、一定時間、y速度が常に同じだと、速度をクリアします。
  else if(fabsf(v_acc[1]) <= VY_MAX_CUT_OFF && v_acc_z[1] == 0){
    // 通り過ぎた分だけ戻す
    dist[1] -= v_acc[1]*s*15.0;
    v_acc[1]=0.0;
  }

  // 低速域で、一定時間、z速度が常に同じだと、速度をクリアします。
  if(fabsf(v_acc[2]) <= VZ_CUT_OFF){
    if((v_acc_z[2] & 0x00ff) == 0){
      // 通り過ぎた分だけ戻す
      dist[2] -= v_acc[2]*s*7.0;
      v_acc[2]=0.0;
   }
  }
  // 高速域で、一定時間、z速度が常に同じだと、速度をクリアします。
  else if(fabsf(v_acc[2]) <= VZ_MAX_CUT_OFF && v_acc_z[2] == 0){
    // 通り過ぎた分だけ戻す
    dist[2] -= v_acc[2]*s*15.0;
    v_acc[2]=0.0;
  }

  // 調整6.
  // IMU を 3軸方向に、個別に、色々、動かして停止させる、移動テスト をします。
  // Rviz 上の tf-base-footprint が暴走しないか、確認します。
  // もし、暴走するようであれば、VX_MAX_CUT_OFF、VY_MAX_CUT_OFF、VZ_MAX_CUT_OFF を増やすか、
  // 今までのアルゴリズムを見直す必要があります。
  // 注) VX_MAX_CUT_OFF、VY_MAX_CUT_OFF、VZ_MAX_CUT_OFF を大きくしすぎると、定速で移動している場合を、
  // 含んでしまうので、要注意です。
  // v_acc_z のマスク数を増やすのも、一手かも!!
  //#define KKKK3
  #ifdef KKKK3
    SERIAL_PORT.print(F("v_acc[0]:"));
    SERIAL_PORT.print(v_acc[0],6);
    SERIAL_PORT.print(F(" v_acc[1]:"));
    SERIAL_PORT.print(v_acc[1],6);
    SERIAL_PORT.print(F(" v_acc[2]:"));
    SERIAL_PORT.println(v_acc[2],6);
  #endif

  // 今回の移動距離(速度*s) ロボット座標系
  dist[0] += v_acc[0]*s;
  dist[1] += v_acc[1]*s;
  dist[2] += v_acc[2]*s;


  // 今回の移動距離(速度*s) を 基準座標系の移動距離に変換
  // 移動距離=v_acc[x y z] * s
  dlt[0]=cb.dt[0][0]*dist[0]+cb.dt[0][1]*dist[1]+cb.dt[0][2]*dist[2];
  dlt[1]=cb.dt[1][0]*dist[0]+cb.dt[1][1]*dist[1]+cb.dt[1][2]*dist[2];
  dlt[2]=cb.dt[2][0]*dist[0]+cb.dt[2][1]*dist[1]+cb.dt[2][2]*dist[2];

  // 基準座標系の距離を積分
  tf_dlt[0] += dlt[0]*10.0;   
  tf_dlt[1] += dlt[1]*10.0;
  tf_dlt[2] += dlt[2]*10.0; 

  //#define KKKK5
  #ifdef KKKK5
    SERIAL_PORT.print(F("tf_dlt[0]:"));
    SERIAL_PORT.print(tf_dlt[0],8);
    SERIAL_PORT.print(F(" tf_dlt[1]:"));
    SERIAL_PORT.println(tf_dlt[1],8);
  #endif
}
#endif

void cIMU::QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw)
{
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

/*
* compCB()
* クォータニオンを回転行列に変換する。
*/
void cIMU::compCB(float q[4],CB *cb){

    double q0q0 = q[0] * q[0];
    double q0q1 = q[0] * q[1];
    double q0q2 = q[0] * q[2];
    double q0q3 = q[0] * q[3];
    double q1q1 = q[1] * q[1];
    double q1q2 = q[1] * q[2];
    double q1q3 = q[1] * q[3];
    double q2q2 = q[2] * q[2];
    double q2q3 = q[2] * q[3];
    double q3q3 = q[3] * q[3];

    //double CB[3][3];
    cb->dt[0][0] = q0q0+q1q1-q2q2-q3q3;
    cb->dt[0][1] = 2.0*(q1q2-q0q3);
    cb->dt[0][2] = 2.0*(q1q3+q0q2);
    cb->dt[1][0] = 2.0*(q1q2+q0q3);
    cb->dt[1][1] = q0q0-q1q1+q2q2-q3q3;
    cb->dt[1][2] = 2.0*(q2q3-q0q1);
    cb->dt[2][0] = 2.0*(q1q3-q0q2);
    cb->dt[2][1] = 2.0*(q2q3+q0q1);
    cb->dt[2][2] = q0q0-q1q1-q2q2+q3q3;
}

/*
* compCB()
* クォータニオンを回転行列に変換する。
*/
void cIMU::compCBd(double q[4],CB *cb){

    double q0q0 = q[0] * q[0];
    double q0q1 = q[0] * q[1];
    double q0q2 = q[0] * q[2];
    double q0q3 = q[0] * q[3];
    double q1q1 = q[1] * q[1];
    double q1q2 = q[1] * q[2];
    double q1q3 = q[1] * q[3];
    double q2q2 = q[2] * q[2];
    double q2q3 = q[2] * q[3];
    double q3q3 = q[3] * q[3];

    //double CB[3][3];
    cb->dt[0][0] = q0q0+q1q1-q2q2-q3q3;
    cb->dt[0][1] = 2.0*(q1q2-q0q3);
    cb->dt[0][2] = 2.0*(q1q3+q0q2);
    cb->dt[1][0] = 2.0*(q1q2+q0q3);
    cb->dt[1][1] = q0q0-q1q1+q2q2-q3q3;
    cb->dt[1][2] = 2.0*(q2q3-q0q1);
    cb->dt[2][0] = 2.0*(q1q3-q0q2);
    cb->dt[2][1] = 2.0*(q2q3+q0q1);
    cb->dt[2][2] = q0q0-q1q1-q2q2+q3q3;
}
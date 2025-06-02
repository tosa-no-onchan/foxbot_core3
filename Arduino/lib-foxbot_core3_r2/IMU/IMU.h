//----------------------------------------------------------------------------
//    프로그램명 	: IMU
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.h
//----------------------------------------------------------------------------
#ifndef _IMU_H_
#define _IMU_H_

#include <inttypes.h>
#include <Arduino.h>

#include <SPI.h>

#include <com_lib.h>
// changed by nishi
//#include "MPU6500.h"

#include "ICM20948.h"
//#include "ICM20948_COMBI.h"

//#include "LSM9DS1.h"

// add by nishi 2025.3.31
//#include "BNO086.h"

//#include "ADAF_BNO086.h"

//#include "ICM42688.h"

#if defined(USE_DUAL_FILTERS)
  #include "MadgwickAHRS.h"
  #include "MahonyAHRS.h"
#elif defined(USE_MADWICK)
  #include "MadgwickAHRS.h"
#elif defined(USE_MADWICK_2)
  #include "MadgwickAHRS_2.h"
#elif defined(USE_MAHONY)
  #include "MahonyAHRS.h"
#endif


// changed by nishi
//#include "imu_selector.h"

#define IMU_OK			  0x00
#define IMU_ERR_I2C		0x01

// 各 IMU で定義する。
//#if defined(ADAF_BNO086_IMU) || defined(ICM20948_COMBI_IMU) || defined(BNO086_IMU)
//  #define IMU_CALI_COUNT_DMP_PRE 100
//  #define IMU_CALI_COUNT_DMP 100
//#else
//  //#define IMU_CALI_COUNT_DMP_PRE 500
//  #define IMU_CALI_COUNT_DMP_PRE 600
//  //#define IMU_CALI_COUNT_DMP 300
//  #define IMU_CALI_COUNT_DMP 400
//#endif

// IMU による、位置の計算
//#define USE_IMU_DIST

struct CB{
  double dt[3][3];
};

#define IMU_MAD_DRIFT_ADJUST_MIN1  30   // 10[min]
#define IMU_MAD_DRIFT_ADJUST_MIN2  60   // 15[min]

class cIMU
{
public:
  // changed by nishi
  #if defined(ICM20948_IMU)
    cICM20948 SEN;
  #elif defined(ICM20948_COMBI_IMU)
    cICM20948_COMBI SEN;
  #elif defined(LSM9DS1_IMU)
    cLSM9DS1 SEN;
  #elif defined(MPU6500_IMU)
  	cMPU6500 SEN;
  #elif defined(BNO086_IMU)
    cBNO086 SEN;
  #elif defined(ADAF_BNO086_IMU)
    cADAF_BNO086 SEN;
  #elif defined(ICM42688_IMU)
    cICM42688 SEN;
  #endif
 
	int16_t angle[3];
  float   rpy[3];
  //float   quat[4];
  // changed by nishi 2025.3.17
  double  quat[4]={1.0, 0.0, 0.0, 0.0};
	double quatRAW[4];		// changed by nishi 2025.3.14
	double quatRAW2[4];		// changed by nishi 2025.3.14
  double  quatZero[4];
  double  quatZeroK[4]={1.0, 0.0, 0.0, 0.0};
  //float   quat_tmp[4];
  // changed by nishi 2025.3.17
  double   quat_tmp[4];
  double   quat2_tmp[4];
  //float   quat_tmp_prev[4];
  // changed by nishi 2025.3.17
  double   quat_tmp_prev[4];
  double  quat_dmp[4];  // add by nishi
  int16_t gyroData[3];
  int16_t gyroRaw[3];
  int16_t accData[3];
  int16_t accRaw[3];
  int16_t magData[3];
  int16_t magRaw[3];

  double ax, ay, az, ax0, ay0, az0;
  float gx, gy, gz, gx0, gy0, gz0;
  float mx, my, mz, mx0, my0, mz0;

	bool bConnected;
  bool start_ok;

  //float aRes;
  //float gRes;
  //float mRes;

  float tf_dlt[3];
  float v_acc[3];
  //float v_acc_2[3];
  uint16_t v_acc_z[3];

  int16_t cali_tf;    // Madgwick Caliburation count
  bool calibratingMad_f; // Madgwick Caliburation count
  int16_t calibratingMad; // Madgwick Caliburation count

  float adjust[3];

  CB cb;


public:
	cIMU();

	//uint8_t  begin( uint32_t hz = 200 );
	//uint8_t  begin( uint32_t hz = 100 );
	//uint8_t  begin( uint32_t hz = 400 );
	uint8_t  begin( uint32_t hz = 800 );
	//uint16_t update( uint32_t option = 0 );
  // changed by nishi 2025.3.7
	bool update( uint32_t option = 0 );

  void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw);

  void compCB(float q[4],CB *cb);
  void compCBd(double q[4],CB *cb);

  // vector math
  float vector_dot(float a[3], float b[3])
  {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  void vector_normalize(float a[3])
  {
    float mag = sqrt(vector_dot(a, a));
    a[0] /= mag;
    a[1] /= mag;
    a[2] /= mag;
  }


private:
  #if defined(USE_DUAL_FILTERS)
    Madgwick filter;
    Mahony filter2;
  #elif defined(USE_MADWICK) || defined(USE_MADWICK_2)
    Madgwick filter;
  #elif defined(USE_MAHONY)
    Mahony filter;
  #endif

  uint32_t update_hz;
  //uint32_t update_us;
  unsigned long update_us;

  uint32_t tTime_t[3];
  uint32_t ac_cnt=0;
  //float zero_off=0.0;


	//double roll,pitch,yaw;
	//double roll_prev,pitch_prev,yaw_prev;


	//void computeIMU( void );
  // changed by nishi 2025.3.7
	bool computeIMU( void );

  //#if defined(ICM20948_IMU)
  // Offsets applied to raw x/y/z mag values
  //float mag_offsets[3]            = { -1.76F, 22.54F, 4.43F };
  // Soft iron error compensation matrix
  //float mag_softiron_matrix[3][3] = { {  0.954,  -0.019,  0.003 },
  //                                    {  -0.019,  1.059, -0.009 },
  //                                    {  0.003,  -0.009,  0.990 } };
  //float mag_field_strength        = 29.85F;
  //#endif

  #if defined(USE_IMU_DIST)
  void computeTF(unsigned long process_time);
  #endif

  
	// IMU Maget キャリブレーション
	// https://www.digikey.fr/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b?srsltid=AfmBOorBMkmk3Q8RwCJZumOZSm4wDxm8KdfLhEoALOvzo42K0nOyuSZI
	// Hard-iron calibration settings
	//float hard_iron[3] = { -32.34,  -1.19,  6.25 };

	// Soft-iron calibration settings
	//float soft_iron[3][3] = { {  0.993,  0.040, -0.002  },
	//	                        {  0.040,  1.003, -0.009  },
	//	                        { -0.002, -0.009,  1.006  }};
	
	// Magnetic declination from magnetic-declination.com
	// East is positive ( ), west is negative (-)
	// mag_decl = ( /-)(deg   min/60   sec/3600)
	// Set to 0 to get magnetic heading instead of geo heading
	//float mag_decl = -1.233;


};


#endif

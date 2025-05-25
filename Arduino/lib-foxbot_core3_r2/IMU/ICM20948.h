/*
* ICM20948.h
*/

#ifndef _ICM20948_H_
#define _ICM20948_H_

#include <inttypes.h>
#include <Arduino.h>
#include "Define.h"
#include "com_lib.h"

#define ICM20948_IMU

// foxboot_core3.ino 用にビルド時は、
// platformio.ini
// -D USE_FOXBOT
// 又は 
// #define USE_FOXBOT

#define USE_FOXBOT

// add by nishi 2023.3.4
//#define USE_TRACE
// platformio.ini
// -D USE_TRACE

// for fooxbot_core3.ino
#if defined(USE_FOXBOT)
	// add by nishi
	#define USE_SPARK_LIB
	#define USE_ACC_NISHI
	#define USE_GRYO_NISHI
	//#define USE_MAG
	//#define USE_DUAL_RATE

	//#define USE_DMP_NISHI
	#define USE_MADWICK
	//#define USE_MADWICK_2
	//#define USE_MAHONY

	// add by nishi 2021.10.7
	#define IMU_SENSER6		// 9軸の時は、コメントにする。

	//#define USE_ACC_2G
	#define USE_ACC_4G
	//#define USE_ACC_8G
	//#define USE_GYRO_250
	#define USE_GYRO_500		// 6軸のときは、こちら。
	//#define USE_GYRO_1000
	//#define USE_GYRO_2000

	#define USE_SCALE_DATA

	//#define USE_IMU_NO1
	//#define USE_IMU_NO2
	#define USE_IMU_NO3

// for micro_ros_tf_publisher
#else
	// add by nishi
	#define USE_SPARK_LIB
	#define USE_ACC_NISHI
	#define USE_GRYO_NISHI
	//#define USE_MAG
	//#define USE_DUAL_RATE

	//#define USE_DMP_NISHI
	//#define USE_AGM_NISHI		// DMP INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)

	#define USE_MADWICK
	//#define USE_MADWICK_2
	//#define USE_MAHONY

	// add by nishi 2021.10.7
	#define IMU_SENSER6		// 9軸の時は、コメントにする。

	//#define USE_ACC_2G
	#define USE_ACC_4G
	//#define USE_ACC_8G
	//#define USE_GYRO_250
	#define USE_GYRO_500		// 6軸のときは、こちら。
	//#define USE_GYRO_1000
	//#define USE_GYRO_2000		// 9軸の時は、こちらか。

	#define USE_SCALE_DATA

	//#define USE_IMU_NO1
	//#define USE_IMU_NO2
	#define USE_IMU_NO3

#endif

#if defined(USE_DUAL_RATE)
	#define MPU_CALI_COUNT_GYRO_PRE 500
	#define MPU_CALI_COUNT_GYRO 500
	#define MPU_CALI_COUNT_ACC_PRE 500
	#define MPU_CALI_COUNT_ACC 500
	#define MPU_CALI_COUNT_MAG 100
	// DMP6 28[Hz]
	#define MPU_CALI_COUNT_DMP 200
	#define MPU_CALI_COUNT_DMP9 200

	// IMU.cpp で使用する。
	#define IMU_CALI_COUNT_DMP_PRE 2000
	#define IMU_CALI_COUNT_DMP 2000
#else
	#define MPU_CALI_COUNT_GYRO_PRE 2000
	#define MPU_CALI_COUNT_GYRO 500
	#define MPU_CALI_COUNT_ACC_PRE 2000
	#define MPU_CALI_COUNT_ACC 500
	#define MPU_CALI_COUNT_MAG 100
	// DMP6 28[Hz]
	#define MPU_CALI_COUNT_DMP 200
	#define MPU_CALI_COUNT_DMP9 200

	// IMU.cpp で使用する。
	#define IMU_CALI_COUNT_DMP_PRE 8000
	#define IMU_CALI_COUNT_DMP 100
#endif

#if defined(IMU_SENSER6)
	#define MADWICK_beta 0.1f
	#define MAHONY_twoKi 1.0f
	//#define MAHONY_twoKi 5.0f		// こちらが、良いみたい
	//#define MAHONY_twoKi 4.0f
#else
	//#define MADWICK_beta 1.0f		// 落ち着くのに時間がかかる。
	#define MADWICK_beta 2.0f
	//#define MAHONY_twoKi 2.0f			// 落ち着くのに時間がかかる。
	#define MAHONY_twoKi 3.0f			//
	//#define MAHONY_twoKi 5.0f
	//#define MAHONY_twoKi 10.0f
	//#define MAHONY_twoKi 20.0f		// 収束しない。
#endif

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
  //  GYRO_FS_SEL=0 ±250 dps 1
  //  GYRO_FS_SEL=1 ±500 dps 1
  //  GYRO_FS_SEL=2 ±1000 dps 1
  //  GYRO_FS_SEL=3 ±2000 dps 1
  // 2.Gyroscope ADC Word Length 16 bits 1
  // 3.Sensitivity Scale Factor
  //  GYRO_FS_SEL=0 131 LSB/(dps) 1
  //  GYRO_FS_SEL=1 65.5 LSB/(dps) 1
  //  GYRO_FS_SEL=2 32.8 LSB/(dps) 1
  //  GYRO_FS_SEL=3 16.4 LSB/(dps) 1
  //
  // Magnet
  // 1.Full-Scale Range ±4900 μT 1
  // 2.Output Resolution 16 bits 1
  // 3.Sensitivity Scale Factor 0.15 μT / LSB 1


#if defined(USE_IMU_NO1)
	#define GYRO_NOISE_CUT_OFF 4
#elif defined(USE_IMU_NO2)
	#define GYRO_NOISE_CUT_OFF 4
#else
	#define GYRO_NOISE_CUT_OFF 10
	//#define GYRO_NOISE_CUT_OFF 8
#endif


#if defined(USE_ACC_2G)
	// 2[G]
	#define ACC_1G 16384.0
	#define ACC_ZERO_OFF 	5.0		// + で上へ行く。 - で下に行く
	#define ACC_X_CUT_OFF 35.0    // 2G  with Low pass filter  ICM20948_ACCEL_BW_6HZ
	#define ACC_Y_CUT_OFF 35.0    // 
	#define ACC_Z_CUT_OFF_P 35.0  // 
	#define ACC_Z_CUT_OFF_M -35.0 // 

    //#define ACC_X_CUT_OFF 300.0    // 2G  without Low pass filter
    //#define ACC_Y_CUT_OFF 300.0    // 
    //#define ACC_Z_CUT_OFF_P 300.0  // 
    //#define ACC_Z_CUT_OFF_M -300.0 // 

    //#define ACC_X_CUT_OFF 150.0    // 2G  without Low pass filter
    //#define ACC_Y_CUT_OFF 150.0    // 
    //#define ACC_Z_CUT_OFF_P 150.0  // 
    //#define ACC_Z_CUT_OFF_M -150.0 // 

	#define ACC_ZERO_Z_OVER 3000

	// 速度
	// 普通にIMU を振って、 0.1 位
	// 低速域の速度を Cut Off
	#define VX_CUT_OFF 0.01
	#define VY_CUT_OFF 0.01
	#define VZ_CUT_OFF 0.01

	// 高速域の速度を Cut Off
	//#define VX_MAX_CUT_OFF 0.35
	//#define VX_MAX_CUT_OFF 0.1
	#define VX_MAX_CUT_OFF 0.08
	//#define VY_MAX_CUT_OFF 0.35
	//#define VY_MAX_CUT_OFF 0.1
	#define VY_MAX_CUT_OFF 0.08
	//#define VZ_MAX_CUT_OFF 0.35
	//#define VZ_MAX_CUT_OFF 0.1
	#define VZ_MAX_CUT_OFF 0.08


#elif defined(USE_ACC_4G)
	// 4[G]
	#define ACC_1G  8192.0
	#define ACC_ZERO_OFF -0.1		// + で上へ行く。 - で下に行く
    #define ACC_X_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    #define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 1500

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	//#define VX_CUT_OFF 0.25
	//#define VX_CUT_OFF 0.08
	#define VX_CUT_OFF 0.05

	//#define VY_CUT_OFF 0.3
	//#define VY_CUT_OFF 0.25
	//#define VY_CUT_OFF 0.08
	#define VY_CUT_OFF 0.05

	//#define VZ_CUT_OFF 0.3
	//#define VZ_CUT_OFF 0.25
	//#define VZ_CUT_OFF 0.08
	#define VZ_CUT_OFF 0.05

#else
	// 8[G]
	#define ACC_1G  4096.0
	#define ACC_ZERO_OFF 0.15		// + で上へ行く。 - で下に行く
    #define ACC_X_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Y_CUT_OFF 20.0    // 4G - 16G
    #define ACC_Z_CUT_OFF_P 20.0  // 4G - 16G
    #define ACC_Z_CUT_OFF_M -20.0 // 4G - 16G

	#define ACC_ZERO_Z_OVER 750

	// 速度を Cut Off
	//#define VX_CUT_OFF 0.3
	//#define VX_CUT_OFF 0.25
	//#define VX_CUT_OFF 0.08
	#define VX_CUT_OFF 0.05

	//#define VY_CUT_OFF 0.3
	//#define VY_CUT_OFF 0.25
	//#define VY_CUT_OFF 0.08
	#define VY_CUT_OFF 0.05

	//#define VZ_CUT_OFF 0.3
	//#define VZ_CUT_OFF 0.25
	//#define VZ_CUT_OFF 0.08
	#define VZ_CUT_OFF 0.05

#endif

// SparkFun_ICM-20948_ArduinoLibrary
#include <ICM_20948.h>  		// Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#if defined(USE_TRACE)
	extern HardwareSerial mySerial2;
	#define SERIAL_PORT mySerial2
#else
	#define SERIAL_PORT Serial
#endif

//  SPARK LIB  use
// for ICM_20948.h
//#define USE_SPI       // Uncomment this to use SPI

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
//#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
//#define CS_PIN 5     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#if defined(BOARD_ESP32)
	// add by nishi
	//#define SS_PIN   5
	#define CS_PIN 5
#elif defined(BOARD_PICO32)
	#define CS_PIN 19
#endif

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
					// On the SparkFun 9DoF IMU breakout the default is 1, and when \
					// the ADR jumper is closed the value becomes 0

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB      (0.15F)

class cICM20948
{

public:
	bool     bConnected;

	int16_t  gyroADC[3];
	float  gyroADC_BD[3];	// add by nishi from ICM-20948
	int16_t  gyroRAW[3];
	int16_t  gyroZero[3];

	int16_t  accADC[3];
	float    accADC_BD[3];	// scale data add by nishi from ICM-20948
	int16_t  accRAW[3];
	int16_t	 accZero[3];
	int32_t	 accZeroSum;
	float    accIMZero;			// add by nishi acc 内積

	int16_t  magADC[3];	// Full +_4900[uT]
	float  magADC_BD[3];	// scale data add by nishi from ICM-20948
	int16_t  magRAW[3];		// magRAW * 0.15 = Full +_4900[uT] , 1 LSB = 0.15[uT]
	int16_t  magZero[3];

	//int32_t  magADC32[3];	// Full +_4900[uT]
	//int32_t  magRAW32[3];		// magRAW * 0.15 = Full +_4900[uT] , 1 LSB = 0.15[uT]
	//int32_t  magZero32[3];

	float mx0,my0,mz0;


	int16_t  gyroData[3];
	int16_t  accSmooth[3];

	uint16_t calibratingG;
	uint16_t calibratingA;
	uint16_t calibratingM;
	uint16_t calibratingD;

	char calibratingG_f;
	char calibratingA_f;
	char calibratingM_f;
	char calibratingD_f;

	// AK8963 get calibration data
	int16_t AK8963_ASA[3];

	double quat[4]={1.0, 0.0, 0.0, 0.0};		// add by nishi
	//int32_t quatRAW[4];		// add by nishi
	double quatRAW[4];		// changed by nishi 2025.3.14
	//int32_t quatZero[4];		// add by nishi
	double quatZero[4];		// changed by nishi 2025.3.14
	double quatZeroK[4]={1.0, 0.0, 0.0, 0.0};		// changed by nishi 2025.3.15

	//aRes = 1.0/16384.0;    // 2g  -> 16384[LSB/g]  ax,ay,az=16384 で、 1[g] を示す。
	//aRes = 1.0/8192.0;     // 4g  -> 8192[LSB/g]	ax,ay,az=8192 で、 1[g] を示す。
	//aRes = 1.0/4096.0;     // 8g  -> 4096[LSB/g]	ax,ay,az=4096 で、 1[g] を示す。
	//aRes = 1.0/2048.0;     // 16g	-> 2048[LSB/g]	ax,ay,az=2048 で、 1[g] を示す。
	float aRes = 1.0/ACC_1G;	//  [G]

	#if defined(USE_GYRO_250)
		float gRes = 1.0/131;	// 250 -> 131 [LSB/dps]
	#elif defined(USE_GYRO_500)
		float gRes = 1.0/65.5;	// 500 -> 65.5 [LSB/dps]
	#elif defined(USE_GYRO_1000)
		float gRes = 1.0/32.8;	// 1000 -> 32.8 [LSB/dps]
	#else
		float gRes = 1.0/16.4;   // 2000dps -> 16.4[LSB/dps] gx,gy,gz=16.4 で、 1[dgree per sec] を示す。
	#endif
	float mRes = 0.15; 		// Sensitivity Scale Factor = 0.15  [uT/LSB]  +_4900 * 0.15 = +_735.0[uT]
	float zero_off;

	uint32_t begin_update_rate = 30;	// begin時の update() calling rate [Hz]

	uint32_t mag_update_rate = 100;		// Magneto の update() calling rate [Hz]
	unsigned long mag_update_rate_us = 1000000/100;	// Magneto の update() calling rate interval us

	unsigned long mag_prev_process_time;
	unsigned long mag_cur_process_time;
	unsigned long mag_process_time;

	bool mag_data_comming;

	// add by nishi 2025.4.12
	// Offsets applied to raw x/y/z mag values
	//float mag_offsets[3]            = { 19.03 , 4.63 , 42.23 };	// MotionCal
	//float mag_offsets[3]            = { 28.67 , -3.63 , 3.34 };	// MotionCali #4 4.7% 2番 10[Hz]
	//float mag_offsets[3]            = { -12.43 , -18.14 , -4.86 };	// MotionCali #5 18.2%     3番 10[Hz]
	//float mag_offsets[3]            = { -18.99 , 8.85 , -1.85 };	// MotionCali #6 22.3%     3番 10[Hz]
	//float mag_offsets[3]            = { 17.76 , 21.86 , 9.71 };	// MotionCali #7 4.5%     2番 10[Hz]
	//float mag_offsets[3]            = { 19.45 , 21.17 , 11.59 };	// MotionCali #8 7.5%     2番 10[Hz]
	//float mag_offsets[3]            = { 18.61 , 21.04 , 9.73 };	// MotionCali #9 4.0%     2番 10[Hz]
	float mag_offsets[3]            = { 19.09 , 22.81 , 9.93 };	// MotionCali #10 2.8%     2番 100[Hz]

	//float mag_offsets[3]            = { 189.57, 235.35, 121.61 };	// calibrate3.py

	// Soft iron error compensation matrix
	//float mag_softiron_matrix[3][3] = {							// MotionCal
	//								{ 0.835 , 0.042 , -0.014 },
	//								{ 0.042 , 0.820 , 0.094 },
	//								{ -0.014 , 0.094 , 1.476 }};

	
	//float mag_softiron_matrix[3][3] = {							// MotionCal #4 4.7%
	//										{ 1.287 , -0.204 , -0.057 },
	//										{ -0.204 , 0.956 , 0.045 },
	//										{ -0.057 , 0.045 , 0.846 }};

	//float mag_softiron_matrix[3][3] = {							// MotionCal #5 18.2%
	//											{ 1.286 , 0.040 , 0.036 },
	//											{ 0.040 , 1.002 , 0.048 },
	//											{ 0.036 , 0.048 , 0.780 }};

	//float mag_softiron_matrix[3][3] = {							// MotionCal #6 22.3%
	//											{ 1.071 , -0.076 , 0.036 },
	//											{ -0.076 , 1.020 , -0.049 },
	//											{ 0.036 , -0.049 , 0.924 }};
												
	//float mag_softiron_matrix[3][3] = {							// MotionCal #7 4.5%
	//											{ 1.059 , -0.055 , 0.006 },
	//											{ -0.055 , 1.024 , -0.051 },
	//											{ 0.006 , -0.051 , 0.928 }};

	//float mag_softiron_matrix[3][3] = {							// MotionCal #8 7.5%
	//											{ 1.084 , -0.017 , -0.006 },
	//											{ -0.017 , 1.002 , -0.070 },
	//											{ -0.006 , -0.070 , 0.926 }};
	
	//float mag_softiron_matrix[3][3] = {							// MotionCal #9 4.0%
	//											{ 1.054 , 0.030 , 0.023 },
	//											{ 0.030 , 0.937 , -0.053 },
	//											{ 0.023 , -0.053 , 1.017 }};

	float mag_softiron_matrix[3][3] = {							// MotionCal #10 2.8%
												{ 1.050 , 0.004 , -0.008 },
												{ 0.004 , 0.974 , -0.057 },
												{ -0.008 , -0.057 , 0.981 }};


	//float mag_softiron_matrix[3][3] = {							// calibrate3.py
	//	{ 1.97511 , -0.02934 , 0.0124 },
	//	{ -0.02934 , 1.91427 , -0.09698 },
	//	{ 0.0124 , -0.09698 , 1.73698 }};


										
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

	bool agm_rcv_f[3];

public:
	cICM20948();

  	bool begin( void );
  	bool init( void );
	bool update( void );
	void gyro_init( void );
	void gyro_get_adc( void );
	void gyro_common();
	void gyro_cali_start();
	bool gyro_cali_get_done();

	void acc_init( void );
	void acc_get_adc( void );
	void acc_common();
	void acc_cali_start();
	bool acc_cali_get_done();

	#if defined(USE_DMP_NISHI)
		void dmp_init( void );
		bool dmp_get_adc();
		bool dmp_cali_get_done();
	#endif

	#if defined(USE_AGM_NISHI)
		bool agm_get_adc();
		void agm_common();
	#endif

	void mag_init( void );
	bool mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();

private:
	#if defined(USE_SPI)
		ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
	#else
		ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
	#endif

	float invSqrt(float x);
};

#endif
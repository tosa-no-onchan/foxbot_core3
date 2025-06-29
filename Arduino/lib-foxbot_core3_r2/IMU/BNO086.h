/*
* BNO086.h
*/

#ifndef _BNO086_H_
#define _BNO086_H_

#include <inttypes.h>
#include <Arduino.h>
#include "Define.h"
#include "com_lib.h"

#define BNO086_IMU

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
	#define USE_WIRE
	// add by nishi
	//#define USE_SPARK_LIB
	//#define USE_ACC_NISHI
	//#define USE_GRYO_NISHI
	//#define USE_MAG

	// ACC and GYRO
	//#define USE_AG_NISHI

	#define USE_DMP_NISHI
	//#define USE_MADWICK
	//#define USE_MAHONY

	// add by nishi 2021.10.7
	#define IMU_SENSER6		// 9軸の時は、コメントにする。

	#define USE_ACC_2G
	//#define USE_ACC_4G
	//#define USE_ACC_8G

// for micro_ros_tf_publisher
#else
	#define USE_WIRE
	// add by nishi
	//#define USE_SPARK_LIB
	//#define USE_ACC_NISHI
	//#define USE_GRYO_NISHI
	//#define USE_MAG

	// ACC and GYRO
	//#define USE_AG_NISHI

	#define USE_DMP_NISHI
	//#define USE_MADWICK
	//#define USE_MAHONY

	// add by nishi 2021.10.7
	#define IMU_SENSER6		// 9軸の時は、コメントにする。

	#define USE_ACC_2G
	//#define USE_ACC_4G
	//#define USE_ACC_8G
#endif

#define use_sen_update

// add by nishi 2025.3.24
#define MPU_CALI_COUNT_GYRO 50
#define MPU_CALI_COUNT_ACC 50
#define MPU_CALI_COUNT_MAG 50
#define MPU_CALI_COUNT_DMP_PRE 300
#define MPU_CALI_COUNT_DMP 200

// IMU.cpp で使用する。
#define IMU_CALI_COUNT_DMP_PRE 100
#define IMU_CALI_COUNT_DMP 100

#if defined(IMU_SENSER6)
	#define MADWICK_beta 0.1f
	#define MAHONY_twoKi 2.0f
#else
	#define MADWICK_beta 5.0f
	#define MAHONY_twoKi 10.0f
#endif


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

#define GYRO_NOISE_CUT_OFF 4

#if defined(USE_ACC_2G)
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

#if defined(USE_WIRE)
  #include <Wire.h>
#else
  #include <SPI.h>
  //#define MPU_SPI   SPI_IMU
  // changed by nishi
  #define MPU_SPI   SPI
#endif


// For SPI, we need some extra pins defined:
// Note, these can be other GPIO if you like.
//#define BNO08X_CS   5
//#define BNO08X_INT  A4
//#define BNO08X_RST  A5

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed


#if defined(BOARD_ESP32)
	//#define BDPIN_SPI_CS_IMU 5
	#define BNO08X_CS   5
	//#define BNO08X_INT  2
	#define BNO08X_INT  -1
	#define BNO08X_RST  15
	
#elif defined(BOARD_PICO32)
	//#define BDPIN_SPI_CS_IMU 19
	#define BNO08X_CS   19
	//#define BNO08X_INT  32
	#define BNO08X_INT  -1
	#define BNO08X_RST  33
#endif

#if defined(USE_TRACE)
	extern HardwareSerial mySerial2;
	#define SERIAL_PORT mySerial2
#else
	#define SERIAL_PORT Serial
#endif

// SparkFun_BNO08x_ArduinoLibrary
#include <SparkFun_BNO08x_Arduino_Library.h>  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

//  SPARK LIB  use
// for ICM_20948.h
//#if defined(USE_SPARK_LIB)
//	#define USE_SPI       // Uncomment this to use SPI

//	//#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
//	//#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
//	//#define CS_PIN 5     // Which pin you connect CS to. Used only when "USE_SPI" is defined
//	#define CS_PIN BDPIN_SPI_CS_IMU     // Which pin you connect CS to. Used only when "USE_SPI" is defined

//	#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
//	#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
						// On the SparkFun 9DoF IMU breakout the default is 1, and when \
						// the ADR jumper is closed the value becomes 0
//#endif

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB      (0.15F)

class cBNO086
{

public:
	bool     bConnected;

	int16_t	gyroADC[3];
	float	gyroADC_BD[3];	// 
	int16_t	gyroRAW[3];
	int16_t	gyroZero[3];
	float	gyroZero_BD[3];

	int16_t	accADC[3];
	float	accADC_BD[3];	// 
	int16_t	accRAW[3];
	int16_t	accZero[3];
	float	accZero_BD[3];
	int16_t	accZeroSum;
	float	accZeroSum_BD;
	float	accIMZero;			// add by nishi acc 内積

	int16_t  magADC[3];		// 
	float  magADC_BD[3];	// 
	int16_t  magRAW[3];		// 
	int16_t  magZero[3];
	float  magZero_BD[3];

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

	float gRes = 1.0/16.4;   // 2000dps -> 16.4[LSB/dps] gx,gy,gz=16.4 で、 1[dgree pre sec] を示す。
	float mRes = 0.15; // Sensitivity Scale Factor = 0.15  [uT/LSB]  +_4900 * 0.15 = +_735.0[uT]
	float zero_off;

	uint32_t begin_update_rate = 40;	// begin時の update() calling rate [Hz]


public:
	cBNO086();

  	bool begin( void );
  	bool init( void );
	bool update( void );
	void gyro_init( void );
	bool gyro_get_adc( void );
	void gyro_common();
	void gyro_cali_start();
	bool gyro_cali_get_done();

	void acc_init( void );
	bool acc_get_adc( void );
	void acc_common();
	void acc_cali_start();
	bool acc_cali_get_done();

	#if defined(USE_DMP_NISHI)
		void dmp_init( void );
		bool dmp_get_adc();
		bool dmp_cali_get_done();
	#endif

	#if defined(USE_AGM_NISHI) || defined(USE_AG_NISHI)
		bool agm_get_adc();
		void agm_common();
	#endif

	void mag_init( void );
	bool mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();

private:
	BNO08x myIMU; // If using SPI create an BNO08x SPI object

	float invSqrt(float x);

};

#endif
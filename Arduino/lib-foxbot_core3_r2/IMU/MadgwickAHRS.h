//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

//extern volatile float beta;				// algorithm gain
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//#define sampleFreq	512.0f		// sample frequency in Hz
//#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Function declarations

class Madgwick{

    public:
        float invSampleFreq;
        float q0,q1,q2,q3;
        float beta = 0.1f;
        //float beta = 1.0f;
        float sampleFreq = 512.0f;		// sample frequency in Hz
        Madgwick(void){};
    
        void begin(void) {
            q0 = 1.0f;
            q1 = 0.0f;
            q2 = 0.0f;
            q3 = 0.0f;
            invSampleFreq = 1.0f / sampleFreq;
        }
        void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    };


#endif
//=====================================================================================================
// End of file
//=====================================================================================================

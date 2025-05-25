//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

class Mahony{

    public:
    //---------------------------------------------------------------------------------------------------
    // Definitions

    //#define sampleFreq	512.0f			// sample frequency in Hz
    //#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
    //#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

    //---------------------------------------------------------------------------------------------------
    // Variable definitions

    //float twoKp = 2.0f * 0.5f;											// 2 * proportional gain (Kp) original 6軸 -> OK
    //float twoKp = 2.0f * 1.0f;											// 2 * proportional gain (Kp) 
    //float twoKp = 2.0f * 1.5f;											// 2 * proportional gain (Kp) 
    //float twoKp = 50.0f;											// 2 * proportional gain (Kp) 
    //float twoKp = 30.0f;											// 2 * proportional gain (Kp) 
    //float twoKp = 15.0f;											// 2 * proportional gain (Kp) 
    float twoKp = 10.0f;											// 2 * proportional gain (Kp) 
    float twoKi = 2.0f * 0.0f;											// 2 * integral gain (Ki) original
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
    float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
    float sampleFreq = 512.0f;		// sample frequency in Hz

    float invSampleFreq;

    Mahony(void){};

    void begin(void) {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        invSampleFreq = 1.0f / sampleFreq;
    }
    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);
};

#endif
//=====================================================================================================
// End of file
//=====================================================================================================

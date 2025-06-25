/*
*  com_lib/com_lib.h
*/
#ifndef _COM_LIB_H_
#define _COM_LIB_H_

#include <Arduino.h>
#include <stdio.h>
#include <time.h>

// python3だと
// import math
// 180/math.pi 
// 180/3.141592653589793 -> 57.29577951308232
// 1[degree] -> 0.0174533[radian]    3.14/180
#ifndef RADIANS_F
#define RADIANS_F   57.29577951308232    // [deg/rad]
#endif

namespace foxbot3{

typedef unsigned long long usec_t;
//typedef unsigned long  usec_t;

struct CB{
  double dt[3][3];
};

inline unsigned long micros_2(){
    //uint32_t m = millis();
    //unsigned long t = micros();
    return micros();
}

inline uint64_t micros_(){
    //uint32_t m = millis();
    //unsigned long t = micros();
    return ((uint64_t)millis()) * 1000 + ((uint64_t)micros())%1000;
}

/*
* T round_my(T dt,int n)
*/
template <class T=float>
T round_my(T dt,int n)
{
    if (n > 0){
        T x=(T)pow(10.0, n);
        dt *= x;
        return std::round(dt) / x;
    }
    else{
        return std::round(dt);
    }
}

/*
* T round_my_zero(T dt,int n)
*/
template <class T=float>
T round_my_zero(T dt)
{
    return round_my<T>(dt,5);
}

/*
* float normalize_tf_rz(float rz)
*  tf rz [radian] の補正
*   float rz: [Radian]
*   180度[Radian] 以上を補正します。  
*  piを超す(WrapAround) と、符号が変わります。
*  TF rz は、常に 0pi から pi の間の角度と回転方向を返す。(0pi から近い角度)
*/
float normalize_tf_rz(float rz);

/*
*  Degree to Quaternion
*/
template <typename T>
void degree_to_quat(T x, T y, T z, double* q) {
    float c1 = cos((y*3.14/180.0)/2);
    float c2 = cos((z*3.14/180.0)/2);
    float c3 = cos((x*3.14/180.0)/2);

    float s1 = sin((y*3.14/180.0)/2);
    float s2 = sin((z*3.14/180.0)/2);
    float s3 = sin((x*3.14/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

/*
*  Rad to Quaternion
*/
template <typename T>
void euler_to_quat(T x, T y, T z, double* q) {
    float c1 = cos(y/2);
    float c2 = cos(z/2);
    float c3 = cos(x/2);

    float s1 = sin(y/2);
    float s2 = sin(z/2);
    float s3 = sin(x/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}


//inline void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
//            double& roll, double& pitch, double& yaw)
inline void QuaternionToEulerAngles(double *q, double& roll, double& pitch, double& yaw)
{
      //double q0q0 = q0 * q0;
      double q0q0 = q[0] * q[0];
      //double q0q1 = q0 * q1;
      double q0q1 = q[0] * q[1];
      //double q0q2 = q0 * q2;
      double q0q2 = q[0] * q[2];
      //double q0q3 = q0 * q3;
      double q0q3 = q[0] * q[3];
      //double q1q1 = q1 * q1;
      double q1q1 = q[1] * q[1];
      //double q1q2 = q1 * q2;
      double q1q2 = q[1] * q[2];
      //double q1q3 = q1 * q3;
      double q1q3 = q[1] * q[3];
      //double q2q2 = q2 * q2;
      double q2q2 = q[2] * q[2];
      //double q2q3 = q2 * q3;
      double q2q3 = q[2] * q[3];
      //double q3q3 = q3 * q3;
      double q3q3 = q[3] * q[3];
      roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
      pitch = asin(2.0 * (q0q2 - q1q3));
      yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}


//// Kakezan add by nishi 2025.3.15
// https://researchmap.jp/blogs/blog_entries/view/77082/74054f82f27b8fe084d5eb7426f55571?frame_id=836719

#if defined(USE_ORG_KAKE)
inline void Kakezan(double *left,   double *right, double *ans)
{      
  //foxbot3::quaternion   ans;
  double   d1, d2, d3, d4;      

  //d1   =  left.t * right.t;
  d1   =  left[0] * right[0];      
  //d2   = -left.x * right.x;
  d2   = -left[1] * right[1];
  //d3   = -left.y * right.y;
  d3   = -left[2] * right[2];
  //d4   = -left.z * right.z;
  d4   = -left[3] * right[3];
  //     ans.t = d1+ d2+ d3+ d4;
        ans[0] = d1+ d2+ d3+ d4;

  //d1   =  left.t * right.x;
  d1   =  left[0] * right[1];
  //d2   =  right.t * left.x;
  d2   =  right[0] * left[1];
  //d3   =  left.y * right.z;
  d3   =  left[2] * right[3];
  //d4   = -left.z * right.y;
  d4   = -left[3] * right[2];
  //     ans.x =  d1+ d2+ d3+ d4;
        ans[1] =  d1+ d2+ d3+ d4;

  //d1   =  left.t * right.y;
  d1   =  left[0] * right[2];
  //d2   =  right.t * left.y;
  d2   =  right[0] * left[2];
  //d3   =  left.z * right.x;
  d3   =  left[3] * right[1];
  //d4   = -left.x * right.z;
  d4   = -left[1] * right[3];
  //   ans.y =  d1+ d2+ d3+ d4;
        ans[2] =  d1+ d2+ d3+ d4;

  //d1   =  left.t * right.z;
  d1   =  left[0] * right[3];
  //d2   =  right.t * left.z;
  d2   =  right[0] * left[3];
  //d3   =  left.x * right.y;
  d3   =  left[1] * right[2];
  //d4   = -left.y * right.x;
  d4   = -left[2] * right[1];
  //     ans.z =  d1+ d2+ d3+ d4;
        ans[3] =  d1+ d2+ d3+ d4;
        
  //return   ans;      
}
#else
template <typename T>
void Kakezan(T *left,   T *right, T *ans)
{      
  //foxbot3::quaternion   ans;
  T   d1, d2, d3, d4;      

  //d1   =  left.t * right.t;
  d1   =  left[0] * right[0];      
  //d2   = -left.x * right.x;
  d2   = -left[1] * right[1];
  //d3   = -left.y * right.y;
  d3   = -left[2] * right[2];
  //d4   = -left.z * right.z;
  d4   = -left[3] * right[3];
  //     ans.t = d1+ d2+ d3+ d4;
        ans[0] = d1+ d2+ d3+ d4;

  //d1   =  left.t * right.x;
  d1   =  left[0] * right[1];
  //d2   =  right.t * left.x;
  d2   =  right[0] * left[1];
  //d3   =  left.y * right.z;
  d3   =  left[2] * right[3];
  //d4   = -left.z * right.y;
  d4   = -left[3] * right[2];
  //     ans.x =  d1+ d2+ d3+ d4;
        ans[1] =  d1+ d2+ d3+ d4;

  //d1   =  left.t * right.y;
  d1   =  left[0] * right[2];
  //d2   =  right.t * left.y;
  d2   =  right[0] * left[2];
  //d3   =  left.z * right.x;
  d3   =  left[3] * right[1];
  //d4   = -left.x * right.z;
  d4   = -left[1] * right[3];
  //   ans.y =  d1+ d2+ d3+ d4;
        ans[2] =  d1+ d2+ d3+ d4;

  //d1   =  left.t * right.z;
  d1   =  left[0] * right[3];
  //d2   =  right.t * left.z;
  d2   =  right[0] * left[3];
  //d3   =  left.x * right.y;
  d3   =  left[1] * right[2];
  //d4   = -left.y * right.x;
  d4   = -left[2] * right[1];
  //     ans.z =  d1+ d2+ d3+ d4;
        ans[3] =  d1+ d2+ d3+ d4;
        
  //return   ans;      
}
#endif


/*
* compCB()
* クォータニオンを回転行列に変換する。
*/
template <typename T>
void compCB(T q[4],CB *cb){

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

}
#endif
#include <Arduino.h>
#include "com_lib.h"

namespace foxbot3{

//// Kakezan add by nishi 2025.3.15
// https://researchmap.jp/blogs/blog_entries/view/77082/74054f82f27b8fe084d5eb7426f55571?frame_id=836719
void Kakezan(double *left,   double *right, double *ans)
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

}
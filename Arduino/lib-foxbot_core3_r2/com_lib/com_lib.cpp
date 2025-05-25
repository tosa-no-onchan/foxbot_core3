#include <Arduino.h>
#include "com_lib.h"

namespace foxbot3{
      /*
      * float normalize_tf_rz(float rz)
      *  tf rz [radian] の補正
      *   float rz: [Radian]
      *   180度[Radian] 以上を補正します。
      *  piを超す(WrapAround) と、符号が変わります。
      *  TF rz は、常に 0pi から pi の間の角度と回転方向を返す。(0pi から近い角度)
      */
      float normalize_tf_rz(float rz){
      //if(abs(rz) >= 360.0/RADIANS_F){
      //    if(rz >0.0)
      //        rz -= 360.0/RADIANS_F;
      //    else
      //        rz += 360.0/RADIANS_F;
      //}
      //if(abs(rz) >= 180.0/RADIANS_F){
      //    if(rz >0.0)
      //        rz -= 180.0/RADIANS_F;
      //    else
      //        rz += 180.0/RADIANS_F;
      //}

      rz= fmod(rz,(360.0/RADIANS_F));

      // 小数点以下5 の 丸めをしないと、うまく行かない。
      //rz = round_my<float>(rz,5);
      rz = round_my_zero<float>(rz);
      if(rz > 180.0/RADIANS_F){
            rz = -360.0/RADIANS_F + rz;
            //std::cout << "normalize_tf_rz() #2 rz=" << rz << " dz="<< rz * RADIANS_F << std::endl;
      }
      else if(rz < -180.0/RADIANS_F){
            rz = 360.0/RADIANS_F + rz;
            //std::cout << "normalize_tf_rz() #3 rz=" << rz << " dz="<< rz * RADIANS_F << std::endl;
      }
      return rz;
      }

}
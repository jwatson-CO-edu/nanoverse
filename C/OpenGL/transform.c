#include "geometry.h"
#include "matrix4x4f.h"


float fvsnf( float angle ){  return 1.0f - cosf( angle );  } // Versine, float version


float* set_matx_rot_axis_angle( float* matx, /*<<*/ const vec4f axis, float angle_rad ){
    // Return a homogeneous transform that is a rotation by `angle_rad` about the `axis`
    /* m0 m4 m8  m12
       m1 m5 m9  m13
       m2 m6 m10 m14
       m3 m7 m11 m15 */
    // 1. Calc components
    vec4f axis_ = unit_vec4f( axis );
    float   k1    = axis_.x;
    float   k2    = axis_.y;
    float   k3    = axis_.z;
    float   vTh   = fvsnf( angle_rad );
    float   cTh   = cosf( angle_rad );
    float   sTh   = sinf( angle_rad );
    // 2. X-basis
    matx[0] = k1*k1*vTh + cTh;
    matx[1] = k2*k1*vTh + k3*sTh;
    matx[2] = k3*k1*vTh - k2*sTh;
    // 3. Y-basis
    matx[ 4] = k1*k2*vTh - k3*sTh;
    matx[ 5] = k2*k2*vTh + cTh;
    matx[ 6] = k3*k2*vTh + k1*sTh;
    // 4. Z-basis
    matx[ 8] = k1*k3*vTh + k2*sTh;
    matx[ 9] = k2*k3*vTh - k1*sTh;
    matx[10] = k3*k3*vTh + cTh;
    // N. Return
    return matx;
}
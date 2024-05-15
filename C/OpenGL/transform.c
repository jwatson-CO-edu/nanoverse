////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// HOMOGENEOUS TRANSFORMATIONS /////////////////////////////////////////////////////////////


void set_posn_mtx44f( float mat[], const vec4f posn ){
    // Set the position components of the homogeneous coordinates
    mat[12] = posn.x;
    mat[13] = posn.y;
    mat[14] = posn.z;
}


vec4f mult_mtx44f_vec4f( float mat[], const vec4f v ){
    // Transform `v` with `mat`
    vec4f rtnV = make_0_vec4f();
    for( int j = 0; j < 4; j++ ){
        rtnV.x += mat[   j] * v.x;
        rtnV.y += mat[ 4+j] * v.y;
        rtnV.z += mat[ 8+j] * v.z;
        rtnV.w += mat[12+j] * v.w;
    }
    // Return
    return rtnV;
}
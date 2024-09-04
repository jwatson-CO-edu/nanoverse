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

vec4f get_posn_mtx44f( const float mat[] ){
    // Get the position components of the homogeneous coordinates
    return make_vec4f( mat[12], mat[13], mat[14] );
}


vec4f mult_mtx44f_vec4f( const float mat[], const vec4f v ){
    // Transform `v` with `mat`
    vec4f rtnV;
    rtnV.x = mat[0]*v.x + mat[4+0]*v.y + mat[8+0]*v.z + mat[12+0]*v.w;
    rtnV.y = mat[1]*v.x + mat[4+1]*v.y + mat[8+1]*v.z + mat[12+1]*v.w;
    rtnV.z = mat[2]*v.x + mat[4+2]*v.y + mat[8+2]*v.z + mat[12+2]*v.w;
    rtnV.w = mat[3]*v.x + mat[4+3]*v.y + mat[8+3]*v.z + mat[12+3]*v.w;
    // Return
    return rtnV;
}
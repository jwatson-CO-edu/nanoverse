// 2024-05-01: This is a revised version of "OGL_Utils.h" and is meant to replace it in the future.
#pragma GCC diagnostic ignored "-Wmissing-braces"

#ifndef OGL_TOOLS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_TOOLS_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes and Defines ////////////////////////////////////////////////
/// Standard ///
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>  
/// Defines ///
#define LEN 8192  // Maximum length of text string
#define GL_GLEXT_PROTOTYPES
/// OpenGL & GLUT ///
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

////////// MATH STRUCTS ////////////////////////////////////////////////////////////////////////////

typedef struct{
  union{ float x; float r; };
  union{ float y; float g; };
  union{ float z; float b; };
  union{ float w; float a; };
} vec4; // TODO: ADD "f" SUFFIX

typedef struct{
    float x;
    float y;
} vec2; // TODO: ADD "f" SUFFIX

typedef struct{
    union{ unsigned int v0; unsigned int f0; };
    union{ unsigned int v1; unsigned int f1; };
    union{ unsigned int v2; unsigned int f2; };
} vec3uu; // TODO: SHORTEN SUFFIX

typedef struct{
    unsigned int ID;
    vec4* /*--*/ arr;
} GPtr_vec4; // TODO: ADD "f" SUFFIX

typedef struct{
    unsigned int ID;
    vec3uu* /**/ arr;
} GPtr_vec3uu; // TODO: SHORTEN SUFFIX

////////// 3D GEOMETRY /////////////////////////////////////////////////////////////////////////////

///// 3D Vectors //////////////////////////////////////////////////////////

vec4 sub_vec4( const vec4 u, const vec4 v ){
    // Calc `u` - `v` = `r`, R^3
    vec4 rtnVec = {
        u.x - v.x,
        u.y - v.y,
        u.z - v.z,
        1.0
    };
    return rtnVec;
}


vec4 add_vec4( const vec4 u, const vec4 v ){
    // Calc `u` + `v` = `r`, R^3
    vec4 rtnVec = {
        u.x + v.x,
        u.y + v.y,
        u.z + v.z,
        1.0
    };
    return rtnVec;
}


float norm_vec4( const vec4 vec ){  
    // Euclidean length of an R^3
    return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); // Assume w = 1.0f
} 


float diff_vec4( const vec4 u, const vec4 v ){  
    // Euclidean length of `u`-`v`
    return norm_vec4( sub_vec4( u, v ) );
} 


vec4 unit_vec4( const vec4 vec ){
    // Calc the unit direction of `vec` and return it, R^3
    vec4  rtnVec;
    float mag = norm_vec4( vec );
    if( mag > 0.0f ){
        rtnVec.x = vec.x / mag;
        rtnVec.y = vec.y / mag;
        rtnVec.z = vec.z / mag;
    }else{
        rtnVec.x = 0.0f;
        rtnVec.y = 0.0f;
        rtnVec.z = 0.0f;
    }
    rtnVec.w = 1.0f;
    return rtnVec;
}

vec4 cross_vec4( const vec4 u, const vec4 v ){
    // Calc `u` X `v` = `p`, R^3
    // Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
    vec4 rtnVec = {
        u.y*v.z - u.z*v.y,
        u.z*v.x - u.x*v.z,
        u.x*v.y - u.y*v.x,
        1.0f
    };
    return rtnVec;
}



vec4 scale_vec4( const vec4 u, float f ){
    // Calc `u` * `f` = `r`, R^3
    vec4 rtnVec = {
        u.x * f,
        u.y * f,
        u.z * f,
        1.0f
    };
    return rtnVec;
}



////////// 2D GEOMETRY /////////////////////////////////////////////////////////////////////////////

///// 2D Vectors //////////////////////////////////////////////////////////

vec2 add_vec2( const vec2 u, const vec2 v ){
    // Calc `u` + `v` = `r`, R^3
    vec2 rtnVec = {
        u.x + v.x,
        u.y + v.y,
    };
    return rtnVec;
}


float norm_vec2( const vec2 vec ){  
    // Euclidean length of an R^2
    return sqrtf(vec.x*vec.x + vec.y*vec.y);  
} 


vec2 scale_vec2( const vec2 u, float f ){
    // Calc `u` * `f` = `r`, R^2
    vec2 rtnVec = {
        u.x * f,
        u.y * f
    };
    return rtnVec;
}


vec2 unit_vec2( const vec2 vec ){
    // Calc the unit direction of `vec` and return it, R^3
    vec2  rtnVec;
    float mag = norm_vec2( vec );
    if( mag > 0.0f ){
        rtnVec.x = vec.x / mag;
        rtnVec.y = vec.y / mag;
    }else{
        rtnVec.x = 0.0f;
        rtnVec.y = 0.0f;
    }
    return rtnVec;
}


vec2 stretch_vec2_to_len( const vec2 vec, float len ){
    // Set `res` to be the same direction as `vec` with `len` 
    return scale_vec2( unit_vec2( vec ), len );
}


////////// 2D <---> 3D /////////////////////////////////////////////////////////////////////////////

vec4 lift_pnt_2D_to_3D( const vec2 pnt2f, const vec4 origin, const vec4 xBasis, const vec4 yBasis ){
    // Project the local 2D point to the global 3D frame
    return add_vec4(
        origin,
        add_vec4(
            scale_vec4( xBasis, pnt2f.x ), 
            scale_vec4( yBasis, pnt2f.y )
        )
    );
}


vec4 lift_vec_2D_to_3D( const vec2 vct2f, const vec4 xBasis, const vec4 yBasis ){
    // Project the local 2D vector to the global 3D frame
    return add_vec4( 
        scale_vec4( xBasis, vct2f.x ), 
        scale_vec4( yBasis, vct2f.y )
    );
}


#endif
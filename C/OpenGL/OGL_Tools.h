// 2024-05-01: This is a revised version of "OGL_Utils.h" and is meant to replace it in the future.

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
    uint  ID;
    vec4* arr;
} GPtr_vec4;

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


float norm_vec4( const vec4 vec ){  
    // Euclidean length of an R^3
    return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); // Assume w = 1.0f
} 


vec4 unit_vec4( const vec4 vec ){
    // Calc the unit direction of `vec` and return it, R^3
    vec4  rtnVec;
    float mag = norm_vec4( vec );
    if( mag > 0.0f ){
        rtnVec.x = vec.x / mag;
        rtnVec.y = vec.y / mag;
        rtnVec.x = vec.x / mag;
    }else{
        rtnVec.x = 0.0f;
        rtnVec.y = 0.0f;
        rtnVec.x = 0.0f;
    }
    rtnVec.w = 1.0f;
    return rtnVec;
}


#endif
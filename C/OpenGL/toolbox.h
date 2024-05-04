#ifndef TOOLBOX_H // This pattern is to prevent symbols to be loaded multiple times
#define TOOLBOX_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>  
#include <stdbool.h> // bool

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

///// Aliases & Defines ///////////////////////////////////////////////////

typedef unsigned char ubyte;
typedef unsigned int  uint;
typedef unsigned long ulong;

#define LEN 8192  // Maximum length of text string
#define GL_GLEXT_PROTOTYPES


////////// VECTOR STRUCTS //////////////////////////////////////////////////////////////////////////

typedef struct{
    union{ float x; float r; };
    union{ float y; float g; };
    union{ float z; float b; };
    union{ float w; float a; };
} vec4f;

typedef struct{
  float x;
  float y;
} vec2f;

typedef struct{
    union{ unsigned int v0; unsigned int f0; };
    union{ unsigned int v1; unsigned int f1; };
    union{ unsigned int v2; unsigned int f2; };
} vec3u;


#endif
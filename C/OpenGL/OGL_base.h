#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" 
#pragma GCC diagnostic ignored "-Wmissing-braces" 

#ifndef OGL_BASE_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_BASE_H // from multiple imports

////////// INCLUDES & DEFINES //////////////////////////////////////////////////////////////////////

///// Includes ////////////////////////////////////////////////////////////
/// Standard ////
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>  
#include <stdbool.h> // bool

/// OpenGL + GLUT ////
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

///// Defines & Flags /////////////////////////////////////////////////////
#define LEN 8192  // Maximum length of text string
#define GL_GLEXT_PROTOTYPES

////////// TYPE DEFINES ////////////////////////////////////////////////////////////////////////////

typedef unsigned char ubyte;
typedef unsigned int  uint;
typedef unsigned long ulong;


#endif
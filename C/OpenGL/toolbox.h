#ifndef TOOLBOX_H // This pattern is to prevent symbols to be loaded multiple times
#define TOOLBOX_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////////////////////////////////////////////////////////////

/// Standard ///
#include <stdlib.h> //_ Standard Library
#include <stdarg.h> //_ Variable Args
#include <stdio.h> // - Console I/O
#include <math.h> // -- `sqrt`, `sin`, `cos`, ...
#include <time.h> // -- For animation / FPS calcs
#include <errno.h> // - Get OS errors 
#include <stdbool.h> // bool

/// OpenGL ///
#include <GL/glut.h> // Basic window drawing for OpenGL
#include <GL/glu.h> //- OpenGL Utils
#include <GL/gl.h> // - OpenGL

///// Aliases & Defines ///////////////////////////////////////////////////

/// Type Aliases ///
typedef unsigned char ubyte;
typedef unsigned int  uint;
typedef unsigned long ulong;

/// Defines & Flags ///
#define LEN 8192  // Maximum length of text string
#define GL_GLEXT_PROTOTYPES



////////// VECTOR STRUCTS //////////////////////////////////////////////////////////////////////////

typedef struct{
    // Basic 3D vector with a scaling factor, `float`
    union{ float x; float r; };
    union{ float y; float g; };
    union{ float z; float b; };
    union{ float w; float a; };
} vec4f;

typedef struct{
    // Basic 2D vector, `float`
    float x;
    float y;
} vec2f;

typedef struct{
    // 3D Indexing vector, For triangle vertices and adjacent faces
    union{ unsigned int v0; unsigned int f0; };
    union{ unsigned int v1; unsigned int f1; };
    union{ unsigned int v2; unsigned int f2; };
} vec3u;


#endif
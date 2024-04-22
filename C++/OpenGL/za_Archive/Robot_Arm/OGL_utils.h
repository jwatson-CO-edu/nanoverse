#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
OGL_utils.h
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

#ifndef OGL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_UTILS_H // from multiple imports

// ~~ Defines ~~
#define LEN 8192  // Maximum length of text string
#define _USE_MATH_DEFINES
//  OpenGL with prototypes for glext
#define GL_GLEXT_PROTOTYPES // Important for all of your programs
// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
#define Cos(x)( cos( (x) * 3.1415927 / 180 ) )
#define Sin(x)( sin( (x) * 3.1415927 / 180 ) )
#define Cosf(x)( (float)cos( (x) * 3.1415927 / 180 ) )
#define Sinf(x)( (float)sin( (x) * 3.1415927 / 180 ) )

// ~~ Standard ~~
#include <stdio.h> // Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function
#include <math.h> // ceilf
#include <stdbool.h> // Why isn't this a part of every language since ever?

// ~~ OpenGL ~~
#ifdef __APPLE__ // This constant is always defined on Apple machines
      #include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
      #include <GL/glut.h>
#endif

// ~~ Local ~~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/

// ~~ Shortcuts and Aliases ~~



// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void Print( const char* format , ... );

void draw_origin( float scale );

void draw_grid_org_XY( float gridSize , uint xPlusMinus , uint yPlusMinus , 
					   float lineThic );
					   
void Vertex_polr( float th , float ph );

void sphere2( float x , float y , float z , float r );

void cube( float x , float y , float z ,
           float dx , float dy , float dz ,
           float fillColor[3] , float lineColor[3] );

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/


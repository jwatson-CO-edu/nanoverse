#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
OGL_utils.h
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

////////// 

#ifndef OGL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_UTILS_H // from multiple imports

#include <stdio.h> // Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function
#include <math.h> // ceilf
#include <stdbool.h> // Why isn't this a part of every language since ever?

// ~~ System-Specific Includes ~~
#include <GL/glut.h>

#define LEN 8192  // Maximum length of text string
//  OpenGL with prototypes for glext
#define GL_GLEXT_PROTOTYPES // Important for all of your programs

// ~~ Shortcuts and Aliases ~~



// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void Print( const char* format , ... );

void draw_origin( float scale );

float** matrix_new_f( size_t rows , size_t cols );
void    matrix_del_f( float** matx , size_t rows );

float* linspace_f( float a , float b , size_t N );

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/


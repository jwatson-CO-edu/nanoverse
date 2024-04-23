#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" // UNKNOWN MAGIC

#ifndef OGL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_UTILS_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////
#include <stdio.h> // Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdlib.h> // defines four variable types, several macros, and various functions for performing general functions. , size_t
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function
#include <math.h>

#include <GL/glut.h>

///// Defines /////
// OpenGL with prototypes for glext
#define GL_GLEXT_PROTOTYPES // Important for all of your programs
#define LEN 8192  // Maximum length of text string
#define _USE_MATH_DEFINES

///// Aliases /////
typedef unsigned int uint;
typedef float /*--*/ vec3f[3];



////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x ){  return cos( (x) * 3.1415927 / 180 );  }
double Sin( double x ){  return sin( (x) * 3.1415927 / 180 );  }
float  Cosf( float x ){  return (float)cos( (x) * 3.1415927 / 180 );  }
float  Sinf( float x ){  return (float)sin( (x) * 3.1415927 / 180 );  }



////////// TEXT / STATUS ///////////////////////////////////////////////////////////////////////////

void Print( const char* format , ... ){
	// Convenience routine to output raster text , Use VARARGS to make this more flexible   
	// Author: Willem A. Schreüder  
	char    buf[ LEN ];
	char*   ch = buf;
	va_list args;
	//  Turn the parameters into a character string
	va_start( args , format );
	vsnprintf( buf , LEN , format , args );
	va_end( args );
	//  Display the characters one at a time at the current raster position
	while( *ch )
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18 , *ch++ );
}



////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

typedef struct {
	// Camera state goes here

	/// Members ///
	vec3f eyeLoc; // ------------ Camera location (world frame)
	vec3f lookPt; // ------------ Focus of camera (world frame)
	vec3f upVctr; // ------------ Direction of "up"

} Camera3D;

/// Methods ///
void look( const Camera3D camera ){
	// Set camera position, target, and orientation
	gluLookAt( (double) camera.eyeLoc[0], (double) camera.eyeLoc[1], (double) camera.eyeLoc[2],  
			   (double) camera.lookPt[0], (double) camera.lookPt[1], (double) camera.lookPt[2],  
			   (double) camera.upVctr[0], (double) camera.upVctr[1], (double) camera.upVctr[2] );
}



////////// MEMORY OPERATIONS ///////////////////////////////////////////////////////////////////////
typedef float matx_Nx3f[][3];
typedef uint  matx_Nx3u[][3];

matx_Nx3f* matrix_new_Nx3f( size_t rows ){
	// Allocate a 2D matrix and return a pointer to it , ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	matx_Nx3f* ptr = malloc( sizeof( float[rows][3] ) );
	return ptr;
}

void matrix_del( float** matx , size_t rows ){
	// Delete a 2D matrix , ROW MAJOR
	for( size_t i = 0 ; i < rows ; i++ ){  free( matx[i] );  }
	free( matx );
}

matx_Nx3u* matrix_new_Nx3u( size_t rows ){
	// Allocate a 2D matrix and return a pointer to it , ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	matx_Nx3u* ptr = malloc( sizeof( uint[rows][3] ) );
	return ptr;
}

void matrix_del( uint** matx , size_t rows ){
	// Delete a 2D matrix , ROW MAJOR
	for( size_t i = 0 ; i < rows ; i++ ){  free( matx[i] );  }
	free( matx );
}

#endif
#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" // UNKNOWN MAGIC

#ifndef OGL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_UTILS_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////
#include <stdio.h> // Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdlib.h> // defines four variable types, several macros, and various functions for performing general functions. , size_t
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function
#include <math.h>
#include <stdbool.h> // bool

#include <GL/glut.h>

///// Defines /////
// OpenGL with prototypes for glext
#define GL_GLEXT_PROTOTYPES // Important for all of your programs
#define LEN 8192  // Maximum length of text string
#define _USE_MATH_DEFINES

///// Aliases /////
/// Basic ///
typedef unsigned int  uint;
typedef unsigned char ubyte;
/// Float Vectors and Matrices ///
typedef float vec2f[2];
typedef float vec3f[3];
typedef float matx_Nx2f[][2];
typedef float matx_Nx3f[][3];
typedef float matx_Nx4f[][4];
/// Unsigned Vectors and Matrices ///
typedef uint vec3u[3];
typedef uint matx_Nx3u[][3];



////////// MATH HELPERS ////////////////////////////////////////////////////////////////////////////
uint min_uint( uint x, uint y ){  return ((x) < (y)) ? (x) : (y);  } 


///// Random Numbers //////////////////////////////////////////////////////
void init_rand(){  srand( time( NULL ) );  }

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf_range( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    float span = hi - lo;
    return lo + span * randf();
}

int randi( int lo, int hi ){
    // Return a pseudo-random number between `lo` and `hi` (int)
    int span = hi - lo;
    return lo + (rand() % span);
}

ubyte rand_ubyte(){
    // Return a pseudo-random unsigned byte
    return (ubyte) randf( 0.0, 256.0 );
}



////////// OPENGL HELPERS //////////////////////////////////////////////////////////////////////////

void glVtx3f( const vec3f v ){  glVertex3f( v[0] , v[1] , v[2] );  } // Set vertex with a vector
void glNrm3f( const vec3f n ){  glNormal3f( n[0] , n[1] , n[2] );  } // Set normal with a vector
void glClr3f( const vec3f c ){  glColor3f(  c[0] , c[1] , c[2] );  } // Set color with a vector



////////// GEOMETRY HELPERS ////////////////////////////////////////////////////////////////////////
void sub( const vec3f* u, const vec3f* v, vec3f* r ){
	// Calc `u` - `v` = `r`, R^3
	(*r)[0] = (*u)[0] - (*v)[0];
	(*r)[1] = (*u)[1] - (*v)[1];
	(*r)[2] = (*u)[2] - (*v)[2];
}

float norm( const vec3f* vec ){  
	// Euclidean length of an R^3
	return sqrtf((*vec)[0]*(*vec)[0] + (*vec)[1]*(*vec)[1] + (*vec)[2]*(*vec)[2]);  
} 

float diff( const vec3f* u, const vec3f* v ){  
	// Euclidean length of `u`-`v`
	vec3f r;  
	sub( u, v, &r );
	return norm( &r );
} 

void unit( const vec3f* vec, vec3f* unt ){
	// Calc the unit direction of `vec` and store in `unt`, R^3
	float mag = norm( vec );
	if( mag > 0.0 ){
		(*unt)[0] = (*vec)[0] / mag;
		(*unt)[1] = (*vec)[1] / mag;
		(*unt)[2] = (*vec)[2] / mag;
	}else{
		(*unt)[0] = 0.0f;
		(*unt)[1] = 0.0f;
		(*unt)[2] = 0.0f;
	}
}

void cross( const vec3f* u, const vec3f* v, vec3f* p ){
	// Calc `u` X `v` = `p`, R^3
	// Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
	(*p)[0] = (*u)[1]*(*v)[2] - (*u)[2]*(*v)[1];
	(*p)[1] = (*u)[2]*(*v)[0] - (*u)[0]*(*v)[2];
	(*p)[2] = (*u)[0]*(*v)[1] - (*u)[1]*(*v)[0];
}

float dot( const vec3f* u, const vec3f* v ){
	// Calc `u` * `v` = `p`, R^3
	// Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
	return (*u)[0]*(*v)[0] + (*u)[1]*(*v)[1] + (*u)[2]*(*v)[2];
}

void tri_center( const vec3f* v0, const vec3f* v1, const vec3f* v2, vec3f* c ){
	// Calc centroid of 3 R^3 points
	// Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
	(*c)[0] = ((*v0)[0] + (*v1)[0] + (*v2)[0]) / 3.0f;
	(*c)[1] = ((*v0)[1] + (*v1)[1] + (*v2)[1]) / 3.0f;
	(*c)[2] = ((*v0)[2] + (*v1)[2] + (*v2)[2]) / 3.0f;
}

void seg_center( const vec3f* v0, const vec3f* v1, vec3f* c ){
	// Calc centroid of 2 R^3 points
	// Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
	(*c)[0] = ((*v0)[0] + (*v1)[0]) / 2.0f;
	(*c)[1] = ((*v0)[1] + (*v1)[1]) / 2.0f;
	(*c)[2] = ((*v0)[2] + (*v1)[2]) / 2.0f;
}


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

matx_Nx3f* matrix_new_Nx3f( size_t rows ){
	// Allocate a 2D matrix and return a pointer to it, ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	matx_Nx3f* ptr = malloc( sizeof( float[rows][3] ) );
	return ptr;
}


matx_Nx2f* matrix_new_Nx2f( size_t rows ){
	// Allocate a 2D matrix and return a pointer to it, ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	matx_Nx2f* ptr = malloc( sizeof( float[rows][2] ) );
	return ptr;
}


matx_Nx4f* matrix_new_Nx4f( size_t rows ){
	// Allocate a 2D matrix and return a pointer to it, ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	matx_Nx4f* ptr = malloc( sizeof( float[rows][4] ) );
	return ptr;
}


void load_3f_to_row( matx_Nx3f* matx, size_t i, float x, float y, float z ){
	// Load an R^3 vector into row `i` of `matx`
	(*matx)[i][0] = x;
	(*matx)[i][1] = y;
	(*matx)[i][2] = z;
}


void load_2f_to_row( matx_Nx2f* matx, size_t i, float x, float y ){
	// Load an R^3 vector into row `i` of `matx`
	(*matx)[i][0] = x;
	(*matx)[i][1] = y;
}


void load_4f_to_row( matx_Nx4f* matx, size_t i, float w, float x, float y, float z ){
	// Load an R^3 vector into row `i` of `matx`
	(*matx)[i][0] = w;
	(*matx)[i][1] = x;
	(*matx)[i][2] = y;
	(*matx)[i][3] = z;
}


void load_vec3f_to_row( matx_Nx3f* matx, size_t i, const vec3f* vec ){
	// Load an R^3 vector into row `i` of `matx`
	(*matx)[i][0] = (*vec)[0];
	(*matx)[i][1] = (*vec)[1];
	(*matx)[i][2] = (*vec)[2];
}


void load_row_to_vec3f( const matx_Nx3f* matx, size_t i, vec3f* vec ){
	(*vec)[0] = (*matx)[i][0];
	(*vec)[1] = (*matx)[i][1];
	(*vec)[2] = (*matx)[i][2];
}


void load_row_to_glVtx3f( const matx_Nx3f* matx, size_t i ){  
	// Set vertex with a matrix row
	glVertex3f( (*matx)[i][0], (*matx)[i][1], (*matx)[i][2] );  
} 


matx_Nx3u* matrix_new_Nx3u( size_t rows ){
	// Allocate a 2D matrix and return a pointer to it , ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	matx_Nx3u* ptr = malloc( sizeof( uint[rows][3] ) );
	return ptr;
}


void load_3u_to_row( matx_Nx3u* matx, size_t i, uint v1, uint v2, uint v3 ){
	// Load an I^3 vector into row `i` of `matx`
	(*matx)[i][0] = v1;
	(*matx)[i][1] = v2;
	(*matx)[i][2] = v3;
}


void load_row_to_vec3u( const matx_Nx3u* matx, size_t i, vec3u* vec ){
	// Load row `i` of `matx` into an I^3 vector
	(*vec)[0] = (*matx)[i][0];
	(*vec)[0] = (*matx)[i][1];
	(*vec)[0] = (*matx)[i][2];
}

#endif
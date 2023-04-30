/***********  
OGL_utils.c
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

#include "OGL_utils.h"

// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

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

void draw_origin( float scale ){
	// Draw scaled axes at origin in [R,G,B] for [X,Y,Z]
	// Adapted from code provided by Willem A. Schreüder   
	
	glBegin( GL_LINES );
		glColor3f( 1 , 0 , 0 ); // R
		glVertex3d( 0     , 0 , 0 ); // Bgn
		glVertex3d( scale , 0 , 0 ); // End
		glColor3f( 0 , 1 , 0 ); // G
		glVertex3d( 0     , 0 , 0 ); // Bgn
		glVertex3d( 0 , scale , 0 ); // End
		glColor3f( 99/255.0 , 133/255.0 , 255/255.0 ); // B-ish
		glVertex3d( 0 , 0 , 0 ); // Bgn 
		glVertex3d( 0 , 0 , scale ); // End
	glEnd();
	
	//  Label axes
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 );
	glRasterPos3d( scale , 0 , 0 ); // Do the next raster operation at the window position corresponding to 3D coords
	Print( "X" );
	glRasterPos3d( 0 , scale , 0 );
	Print( "Y" );
	glRasterPos3d( 0 , 0 , scale );
	Print( "Z" );
}

float** matrix_new_f( size_t rows , size_t cols ){
	// Allocate a 2D matrix and return a pointer to it , ROW MAJOR
	// ALERT: 'malloc' without 'delete'
	float** ptr = (float**)malloc( rows * sizeof( float* ) );
	for( size_t i = 0 ; i < rows ; ++i ){  ptr[i] = (float*)malloc( cols * sizeof( float ) );  }
	return ptr;
}

void matrix_del_f( float** matx , size_t rows ){
	// Delete a 2D matrix , ROW MAJOR
	for( size_t i = 0 ; i < rows ; i++ ){  free( matx[i] );  }
	free( matx );
}

float* linspace_f( float a , float b , size_t N ){
	// Return a pointer to a float array with 'N' elements from 'a' to 'b', inclusive
	// NOTE: If N = 1 , A vector with only 'a' will be returned
	// NOTE: If N = 0 , A NULL pointer is returned
	// ALERT: 'malloc' without 'delete'
	float* xs = NULL;
	if( N == 1 ){  
		xs = (float*)malloc( sizeof( float ) );
		xs[0] = a;
	}else if( N > 1 ){
		float  h   = ( b - a ) / (float)( N - 1 );
		xs  = (float*)malloc( N * sizeof( float ) );
		float  val = a;
		for( size_t i = 0 ; i < N ; i++ ){
			xs[i] = val;
			val += h;
		}
	}
    return xs;
}

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

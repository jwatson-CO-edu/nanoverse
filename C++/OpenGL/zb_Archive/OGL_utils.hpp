/***********  
OGL_utils.h
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

#ifndef OGL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_UTILS_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

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
#include <iostream>
using std::cout, std::endl, std::flush, std::ostream;
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function
#include <math.h> // ceilf
#include <stdbool.h> // Why isn't this a part of every language since ever?
#include <string> 
using std::string;

// ~~ OpenGL ~~
#include <GL/glut.h>

///// Aliases /////
typedef unsigned char  ubyte;
typedef array<float,2> vec2f;
typedef array<float,3> vec3f;

/////////// HELPER FUNCTIONS ///////////////////////////////////////////////////////////////////////

///// Printing Functions //////////////////////////////////////////////////
ostream& operator<<( ostream& os, const vec3f& vec ) { // ostream '<<' operator for vectors
	// NOTE: This function assumes that the ostream '<<' operator for T has already been defined
	os << "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]";
	return os; // You must return a reference to the stream!
}

///// std <--> OpenGL /////////////////////////////////////////////////////
void glVec3f( const vec3f& v ){  glVertex3f( v[0] , v[1] , v[2] );  }
void glClr3f( const vec3f& c ){  glColor3f( c[0] , c[1] , c[2] );  }

///// Random Numbers //////////////////////////////////////////////////////
void init_rand(){  srand( time( NULL ) );  }

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf( float lo, float hi ){
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



/////////// GRAPHICS STRUCTS ///////////////////////////////////////////////////////////////////////
enum VIEWMODE{ ORTHO, PERSP };

struct OGL_window{
	// Manage window things
	
	/// Members ///
	VIEWMODE CURRVIEW; // View type
	GLdouble dim; // ?? Dimension of orthogonal box ??
	GLdouble w2h; // Width:Height Ratio
	GLdouble fov; // Field Of View angle [deg]

	/// Constructor ///
	OGL_window( VIEWMODE view = PERSP ){
		CURRVIEW = view;
		dim /**/ =  1; 
		w2h /**/ =  1; 
		fov /**/ = 55; 
	}

	/// Methods ///
	void create( int width , int height, string name = "WINDOW" ){
		// Actually create the window, WARNING: Call AFTER `glutInit( &argc , argv )`

		//  Request W x H pixel window
		glutInitWindowSize( width, height );
		//  Create the window
		glutCreateWindow( name.c_str() );
	}

	void Project(){
		// Set projection
		// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
		
		//  Tell OpenGL we want to manipulate the projection matrixrestart
		glMatrixMode( GL_PROJECTION );
		//  Undo previous transformations
		glLoadIdentity();
		
		switch( CURRVIEW ){
			case ORTHO:
				//  aspect ratio of the window
				glOrtho( -dim * w2h , +dim * w2h , 
						 -dim       , +dim       , 
						 -dim       , +dim       );
			
				break;
			case PERSP:
				gluPerspective( (double) fov , // -- Field of view angle, in degrees, in the y direction.
								(double) w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
								(double) dim/4.0 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
								(double) 4.0*dim ); // Specifies the distance from the viewer to the far clipping plane (always positive).
				break;
		}
		
		// Switch back to manipulating the model matrix
		glMatrixMode( GL_MODELVIEW );
		// Undo previous transformations
		glLoadIdentity();
	}

};


struct Camera3D{
	// Camera state goes here

	/// Members ///
	vec3f eyeLoc; // ------------ Camera location (world frame)
	vec3f lookPt; // ------------ Focus of camera (world frame)
	vec3f upVctr; // ------------ Direction of "up"0f};
    // vec3f linClr = {0.0f,0.0f,0.0f};

	/// Constructor ///
	Camera3D(){
		eyeLoc = {1.0f, 1.0f, 1.0f};
		lookPt = {0.0f, 0.0f, 0.0f};
		upVctr = {0.0f, 0.0f, 1.0f}; // Up is +Z, I WILL FIGHT YOU
	}

	/// Methods ///
	void look(){
		// Set camera position, target, and orientation
		gluLookAt( (double) eyeLoc[0], (double) eyeLoc[1], (double) eyeLoc[2],  
			       (double) lookPt[0], (double) lookPt[1], (double) lookPt[2],  
			       (double) upVctr[0], (double) upVctr[1], (double) upVctr[2] );
	}

	void set_position( const vec3f& loc ){
		// Move the camera to `loc` and set view
		eyeLoc = loc;
		look();
	}

	void set_target( const vec3f& target ){
		// Point the camera at `target` and set view
		lookPt = target;
		look();
	}
};


/////////// GRAPHICS HELPERS ///////////////////////////////////////////////////////////////////////

bool ErrCheck( const char* where ){
	// See if OpenGL has raised any errors
	// Author: Willem A. (Vlakkies) Schreüder  
	int err = glGetError();
	if( err ){  
		fprintf( stderr , "ERROR: %s [%s]\n" , gluErrorString( err ) , where );  
		return true;
	}else{  return false;  }
}


// void OGL_frame_start(){
// 	// Do this before drawing anything
// 	// Clear the image
// 	glClearDepth( 1.0 );
// 	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
// 	// Reset previous transforms to the identity matrix
// 	glLoadIdentity();
// }


// void OGL_frame_end(){
// 	// Do this after drawing everything, Flush and swap
// 	ErrCheck( "display" );
// 	glFlush();
// 	glutSwapBuffers();
// }


void Print( const char* format , ... ){
	// Convenience routine to output raster text , Use VARARGS to make this more flexible   
	// Author: Willem A. (Vlakkies) Schreüder  
	char    buf[ LEN ];
	char*   ch = buf;
	va_list args;
	//  Turn the parameters into a character string
	va_start( args , format );
	vsnprintf( buf , LEN , format , args );
	va_end( args );
	//  Display the characters one at a time at the current raster position
	while( *ch ){
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18 , *ch++ );
	}
}


void Print( string format , ... ){
	// 'std::string' version of the above
	va_list args;
	va_start( args , format );
	Print( format.c_str() , args );
	va_end( args );
}


void draw_origin( float scale ){
	//  Draw scaled axes at origin in [R,G,B] for [X,Y,Z]
	glLineWidth( 1.5 );
	
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
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	glRasterPos3d( scale , 0 , 0 ); // Do the next raster operation at the window position corresponding to 3D coords
	Print( "X" );
	glRasterPos3d( 0 , scale , 0 );
	Print( "Y" );
	glRasterPos3d( 0 , 0 , scale );
	Print( "Z" );
}

void draw_grid_org_XY( float gridSize , uint xPlusMinus , uint yPlusMinus , 
					   float lineThic ){
	// Draw a square grid centered at the origin, extending 'xPlusMinus' units in X and 'yPlusMinus' units in Y
	
	float xMin = - gridSize * xPlusMinus , 
		  xMax =   gridSize * xPlusMinus ,
		  yMin = - gridSize * yPlusMinus , 
		  yMax =   gridSize * yPlusMinus ;
	
	glLineWidth( lineThic );
	glColor3f( 1 , 1 , 1 ); 
	
	glBegin( GL_LINES );
	
		// 1. Draw the axis , X
		glVertex3d( 0 , yMin , 0 ); // Bgn
		glVertex3d( 0 , yMax , 0 ); // End
		// 2. Draw the axis , Y
		glVertex3d( xMin , 0 , 0 ); // Bgn
		glVertex3d( xMax , 0 , 0 ); // End
		
		// 3. Draw the grid , X
		for( uint i = 0 ; i < xPlusMinus ; i++ ){
			// Plus
			glVertex3d(  gridSize * i , yMin , 0 ); // Bgn
			glVertex3d(  gridSize * i , yMax , 0 ); // End
			// Minus
			glVertex3d( -gridSize * i , yMin , 0 ); // Bgn
			glVertex3d( -gridSize * i , yMax , 0 ); // End
		}
		
		// 3. Draw the grid , Y
		for( uint i = 0 ; i < yPlusMinus ; i++ ){
			// Plus
			glVertex3d( xMin ,  gridSize * i , 0 ); // Bgn
			glVertex3d( xMax ,  gridSize * i , 0 ); // End
			// Minus
			glVertex3d( xMin , -gridSize * i , 0 ); // Bgn
			glVertex3d( xMax , -gridSize * i , 0 ); // End
		}
		
	glEnd();
}






#endif


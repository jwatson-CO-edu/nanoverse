/***********  
OGL_utils.h
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

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
#include <string> 
using std::string;

// ~~ OpenGL ~~
#include <GL/glut.h>

// ~~ Local ~~

/////////// HELPER FUNCTIONS ///////////////////////////////////////////////////////////////////////

int    randrange( int end ){    return (int)( rand() % end );    }
size_t randrange( size_t end ){ return (size_t)( rand() % end ); }
int    randrange( int bgn , int end ){ return bgn + (int)( rand() % ( end - bgn ) ); }

void init_rand(){  srand( time( NULL ) );  }


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

		//  Request double buffered, true color window 
		glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
		//  Request W x H pixel window
		glutInitWindowSize( width, height );
		//  Create the window
		glutCreateWindow( name.c_str() );
		// Don't ask questions!
		glEnable( GL_DEPTH_TEST );
		glDepthRange( 0.0f , 1.0f );
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
				gluPerspective( fov , // -- Field of view angle, in degrees, in the y direction.
								w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
								dim/4 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
								4*dim ); // Specifies the distance from the viewer to the far clipping plane (always positive).
				break;
		}
		
		// Switch back to manipulating the model matrix
		glMatrixMode( GL_MODELVIEW );
		// Undo previous transformations
		glLoadIdentity();
	}

};


/////////// GRAPHICS HELPERS ///////////////////////////////////////////////////////////////////////

void OGL_frame_start(){
	// Do this before drawing anything
	// Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	// Reset previous transforms to the identity matrix
	glLoadIdentity();
}


void OGL_frame_end(){
	// Do this after drawing everything, Flush and swap
	glFlush();
	glutSwapBuffers();
}

void Print( const char* format , ... ){
	// Convenience routine to output raster text , Use VARARGS to make this more flexible   
	// Author: Willem A. (Vlakkies) Schreüder  
	char    buf[ LEN ];
	char*   ch = buf;
	va_list args;
	// Turn the parameters into a character string
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

void Vertex_sphr( float th , float ph ){
	// Draw vertex in polar coordinates
	// Author: Willem A. (Vlakkies) Schreüder  
	// glColor3f( Cos( th )*Cos( th ) , Sin(ph)*Sin(ph) , Sin(th)*Sin(th));
	glVertex3d( Sinf( th ) * Cosf( ph ) , 
				Sinf( ph ) , 
				Cosf( th ) * Cosf( ph ) );
}

void sphere2( float x , float y , float z , float r ){
	// Draw a sphere (version 2) at (x,y,z) radius (r)
	// Author: Willem A. (Vlakkies) Schreüder  
	const int d = 5;
	int       th , ph;

	//  Save transformation
	glPushMatrix();
	//  Offset and scale
	glTranslated( x , y , z );
	glScaled( r , r , r );

	//  Latitude bands
	for( ph = -90 ; ph < 90 ; ph += d ){
		glBegin( GL_QUAD_STRIP );
		for( th = 0 ; th <= 360 ; th += d ){
			Vertex_sphr( th , ph     );
			Vertex_sphr( th , ph + d );
		}
		glEnd();
	}

	//  Undo transformations
	glPopMatrix();
}

void cube( float x , float y , float z ,
           float dx , float dy , float dz ,
           float fillColor[3] , float lineColor[3] ){
	// Draw a cube at (x,y,z) dimensions (dx,dy,dz) 
	float lineOffset = 1.005;
	//  Save transformation
	glPushMatrix();
	glColor3f( fillColor[0] , fillColor[1] , fillColor[2] );
	//  Offset
	glTranslated( x , y , z );
	glScaled( dx/2.0f , dy/2.0f , dz/2.0f );
	
	//  Cube
	glBegin(GL_QUADS);
		//  Front
		glVertex3f(-1,-1, 1);
		glVertex3f(+1,-1, 1);
		glVertex3f(+1,+1, 1);
		glVertex3f(-1,+1, 1);
		//  Back
		glVertex3f(+1,-1,-1);
		glVertex3f(-1,-1,-1);
		glVertex3f(-1,+1,-1);
		glVertex3f(+1,+1,-1);
		//  Right
		glVertex3f(+1,-1,+1);
		glVertex3f(+1,-1,-1);
		glVertex3f(+1,+1,-1);
		glVertex3f(+1,+1,+1);
		//  Left
		glVertex3f(-1,-1,-1);
		glVertex3f(-1,-1,+1);
		glVertex3f(-1,+1,+1);
		glVertex3f(-1,+1,-1);
		//  Top
		glVertex3f(-1,+1,+1);
		glVertex3f(+1,+1,+1);
		glVertex3f(+1,+1,-1);
		glVertex3f(-1,+1,-1);
		//  Bottom
		glVertex3f(-1,-1,-1);
		glVertex3f(+1,-1,-1);
		glVertex3f(+1,-1,+1);
		glVertex3f(-1,-1,+1);
	//  End
	glEnd();
	
	// Draw outline
	float d = lineOffset;
	glColor3f( lineColor[0] , lineColor[1] , lineColor[2] );
	
	glBegin( GL_LINES );
		// Bottom
		glVertex3f(-d,-d,-d);
		glVertex3f(+d,-d,-d);
		
		glVertex3f(+d,-d,-d);
		glVertex3f(+d,+d,-d);
		
		glVertex3f(+d,+d,-d);
		glVertex3f(-d,+d,-d);
		
		glVertex3f(-d,+d,-d);
		glVertex3f(-d,-d,-d);

		// Top
		glVertex3f(-d,-d,+d);
		glVertex3f(+d,-d,+d);
		
		glVertex3f(+d,-d,+d);
		glVertex3f(+d,+d,+d);
		
		glVertex3f(+d,+d,+d);
		glVertex3f(-d,+d,+d);
		
		glVertex3f(-d,+d,+d);
		glVertex3f(-d,-d,+d);
		
		// Sides
		glVertex3f(-d,-d,+d);
		glVertex3f(-d,-d,-d);
		
		glVertex3f(+d,-d,+d);
		glVertex3f(+d,-d,-d);
		
		glVertex3f(+d,+d,+d);
		glVertex3f(+d,+d,-d);
		
		glVertex3f(-d,+d,+d);
		glVertex3f(-d,+d,-d);
		
	glEnd();
	//  Undo transformations
	glPopMatrix();
}




#endif


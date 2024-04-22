#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" // 'glWindowPos2i' you clearly got imported, so why you complain?

/*****************************
 HW2.c
 James Watson, 2018 September
 Demonstrates the Lorenz Attractor as a special case of a Lorenz System
 *****************************/ 
/* 
~~ DEV PLAN ~~
[Y] Set up source templates
[Y] Import the convenience function
[Y] Display axes
[Y] Display attractor
	[Y] If the window resizes, the attractor should not distort
	[Y] If the view rotates, the attractor should not change
[ ] A COOL THING (choose one)
	[ ] Colored wavy thing as a ribbon perpendiculator to curvature
		[ ] Test marble
		[ ] Calculate normal vector, tangent vector, and perp vector
		[ ] User drives a pulse up and down the wire , INTERACTIVE! [a,d]
			* Some default ribbon height is changed with the "cursor" is over it
		[ ] Color relates to WHAT?
	{ } Use can change params (streth goal)
	* First-person roller-coaster
	* Visualization of 2D flow - TOO COMPLEX
	* Acceleration / Curvature at a moving point along the curve - See above
	
{{ }} Switch to C++ , FUTURE , agents / objects
*/

/*
 *  arrows Change view angle
 *  0      Reset view angle
 *  ESC    Exit
 */
#include <stdio.h> // Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdlib.h> // defines four variable types, several macros, and various functions for performing general functions. , size_t
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function

// ~~ System-Specific Includes ~~
#ifdef __APPLE__ // This constant is always defined on Apple machines
      #include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
	//#include <GL/gl.h>
	#include <GL/glut.h>
	//#include <GL/glext.h>
#endif

//  OpenGL with prototypes for glext
#define GL_GLEXT_PROTOTYPES // Important for all of your programs

#include "OGL_utils.h"

// === GLOBALS ===

// ~~ Data ~~
float** FUNCTRACE = NULL;
size_t  N_FUNC    = 0;

// ~~ Params ~~
float SIGMA = 10.0;
float RHO   = 28.0;
float BETA  =  2.6666;

// ~~ Control ~~
char* MSG[] = { "Edit SIGMA" , "Edit RHO" , "Edit BETA" , ""     }; // Messages
enum  editMode{ edSIGMA      , edRHO      , edBETA      , edNone }; // Modes
int   CURRMODE    = edNone;
bool  PARAMCHANGE = false;
float INCR        = 0.1; // Increment to change the parameters

// ___ END ___


// === FUNCTIONS ===========================================================================================================================

void lorenz_system_trace( float*** outputCoords , float sigma , float rho , float beta , float T , float dt , size_t* outN ){
	// Plot the [x,y,z] evolution of the lorenz System over time
	// NOTE: This function assumes that 'outputCoords' has been FREED
	// Adapted from "lorenz.c", provided by Willem Schreuder

	// ~ System Parameters ~
	float s = sigma; 
	float r = rho; 
	float b = beta; 
	
	size_t i;
	/*  Coordinates  */
	float x = 1;	float dx = 0.0;
	float y = 1;	float dy = 0.0;
	float z = 1;	float dz = 0.0;

	size_t N  = (size_t)ceilf( T / dt );
	*outN     = N;
	float* Ts = linspace_f( 0 , T , N );
	
	*outputCoords = matrix_new_f( N , 4 );
	
	/*
	*  Integrate N steps 
	*  Explicit Euler integration
	*/
	for( i = 0 ; i < N ; i++ ){
		dx = s * ( y - x );
		dy = x * ( r - z ) - y;
		dz = x * y - b * z;
						(*outputCoords)[i][0] = Ts[i];
		x += dt * dx;	(*outputCoords)[i][1] = x;
		y += dt * dy;	(*outputCoords)[i][2] = y;
		z += dt * dz;	(*outputCoords)[i][3] = z;
		
		if( i % 1000 == 0 ){  
			printf( "%5zu %8.3f %8.3f %8.3f %8.3f\n" , i , 
				(*outputCoords)[i][0] ,
				(*outputCoords)[i][1] ,
				(*outputCoords)[i][2] ,
				(*outputCoords)[i][3] 
			);  
		}
	}
}

void draw_XYZ_timeseries_GL_LINES_f( float*** outputCoords , size_t N ){
	// Render a time series of [ ... , [ t_i , x_i , y_i , z_i ]  , ...  ] as a series of line segments
	
	float* curr = (float*)malloc( 3 * sizeof( float ) );
	float* last = (float*)malloc( 3 * sizeof( float ) );
	last = (*outputCoords)[0];
	
	//  Draw axes in white
	glColor3f( 1 , 1 , 1 );
	
	glBegin( GL_LINES );
	for( size_t i = 1 ; i < N ; i++ ){
		curr = (*outputCoords)[i];
		glVertex3d( last[1] , last[2] , last[3] ); // Bgn
		glVertex3d( curr[1] , curr[2] , curr[3] ); // End
		last = curr;
	}
	glEnd();
}

// ___ END FUNC ____________________________________________________________________________________________________________________________


// === VARIABLES ===========================================================================================================================

//  Globals
int    th     =  60; // Azimuth of view angle
int    ph     = 220; // Elevation of view angle
double dim    =  40; // Dimension of orthogonal box

// ___ END VAR _____________________________________________________________________________________________________________________________


// === DRAWING =============================================================================================================================

void display(){
	// Display the scene
	// Adapted from code provided by Willem Schreuder
	
	//  Clear the image
	glClear( GL_COLOR_BUFFER_BIT );
	
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	//  Set view angle
	glRotated( ph , 1 , 0 , 0 ); // 2. Rotate around the X axis
	glRotated( th , 0 , 1 , 0 ); // 1. Rotate around the Y axis

	draw_XYZ_timeseries_GL_LINES_f( &FUNCTRACE , N_FUNC );
	
	draw_origin( dim / 3.0 );

	//  Display parameters
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	
	Print( "sigma %2.3f, rho %2.3f, beta %2.3f - %s |  theta %i , phi %i" , SIGMA , RHO , BETA , MSG[ CURRMODE ] , th , ph );

	//  Flush and swap
	glFlush();
	glutSwapBuffers();
}

// ___ END DRAW ____________________________________________________________________________________________________________________________


// === INTERACTION =========================================================================================================================

void key( unsigned char ch , int x , int y ){
	// GLUT calls this routine when a key is pressed
	//  Exit on ESC
	// Adapted from code provided by Willem Schreuder
	
	switch( ch ){
		
		case 27 : // [Esc] : Exit
			exit( 0 );
			break;
			
		case '0' : // 0 : Set view angles to 0
			th = ph = 0;
			printf( "theta and phi reset!\n" );
			break;
		
		// {s,r,b,n} : Set the edit mode
			
		case 's' :
			CURRMODE = edSIGMA;
			break;
			
		case 'r' :
			CURRMODE = edRHO;
			break;
			
		case 'b' :
			CURRMODE = edBETA;
			break;
			
		case 'n' :
			CURRMODE = edNone;
			break;
		
		// {+,-} : Increment or decrement the Lorenz System parameter designated by the mode
		//         Then recalc. ONLY recalc when absolutely necessary!
		
		case '=' :	
			switch( CURRMODE ){
				case edSIGMA :
					SIGMA += INCR;
					break;
				case edRHO :
					RHO   += INCR;
					break;
				case edBETA :
					BETA  += INCR;
					break;
			}
			matrix_del_f( FUNCTRACE , N_FUNC );
			lorenz_system_trace( &FUNCTRACE , SIGMA , RHO , BETA , 50 , 0.001 , &N_FUNC );
			break;
			
		case '-' :	
			switch( CURRMODE ){
				case edSIGMA :
					SIGMA -= INCR;
					break;
				case edRHO :
					RHO   -= INCR;
					break;
				case edBETA :
					BETA  -= INCR;
					break;
			}
			matrix_del_f( FUNCTRACE , N_FUNC );
			lorenz_system_trace( &FUNCTRACE , SIGMA , RHO , BETA , 50 , 0.001 , &N_FUNC );
			break;
			
		// <?> : Keys are nice, I guess!
			
		default :
			printf( "There is no function for this key!\n" );
		
	}
	
	//  Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}

void special( int key , int x , int y ){
	// GLUT calls this routine when an arrow key is pressed
	// Adapted from code provided by Willem Schreuder
	
	//  Right arrow key - increase azimuth by 5 degrees
	if( key == GLUT_KEY_RIGHT )
		th += 5;
	//  Left arrow key - decrease azimuth by 5 degrees
	else if( key == GLUT_KEY_LEFT )
		th -= 5;
	//  Up arrow key - increase elevation by 5 degrees
	else if( key == GLUT_KEY_UP )
		ph += 5;
	//  Down arrow key - decrease elevation by 5 degrees
	else if ( key == GLUT_KEY_DOWN )
		ph -= 5;
	//  Keep angles to +/-360 degrees
	th %= 360;
	ph %= 360;
	//  Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}

void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
	// Code provided by Willem Schreuder
	
	//  Ratio of the width to the height of the window
	double w2h = ( height > 0 ) ? (double) width / height : 1;
	//  Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	//  Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	//  Orthogonal projection box adjusted for the
	//  aspect ratio of the window
	glOrtho( -dim * w2h , +dim * w2h , 
			 -dim       , +dim       , 
			 -dim       , +dim       );
	//  Switch to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	//  Undo previous transformations
	glLoadIdentity();
}

// ___ END INTERACT ________________________________________________________________________________________________________________________


// === MAIN ================================================================================================================================

// Start up GLUT and tell it what to do
int main( int argc , char* argv[] ){
	
	// Calc the initial system before the user changes it
	lorenz_system_trace( &FUNCTRACE , SIGMA , RHO , BETA , 50 , 0.001 , &N_FUNC );
	
	//  Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	//  Request double buffered, true color window 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
	
	//  Request 500 x 500 pixel window
	glutInitWindowSize( 1000 , 750 );
	
	//  Create the window
	glutCreateWindow( "James Watson , HW2" );
	
	//  Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );
	
	//  Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );
	
	//  Tell GLUT to call "special" when an arrow key is pressed
	glutSpecialFunc( special );
	
	//  Tell GLUT to call "key" when a key is pressed
	glutKeyboardFunc( key );
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	// Free memory
	matrix_del_f( FUNCTRACE , N_FUNC );
	
	//  Return code
	return 0;
}

// ___ END MAIN ____________________________________________________________________________________________________________________________

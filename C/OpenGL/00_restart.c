// gcc -O3 -Wall 00_restart.c -lglut -lGLU -lGL -lm -o test.out
#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" // UNKNOWN MAGIC

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "OGL_Geo.h"



////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////
int    th     =  60; // Azimuth of view angle
int    ph     = 220; // Elevation of view angle

void display(){
	// Display the scene
	// Adapted from code provided by Willem Schreuder
	
    vec3f cubClr = {0.0f,1.0f,0.0f};
    vec3f linClr = {0.0f,0.0f,0.0f};

	//  Clear the image
	glClear( GL_COLOR_BUFFER_BIT );
	//  Reset previous transforms to the identity matrix
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	//  Set view angle
	glRotated( ph , 1 , 0 , 0 ); // 2. Rotate around the X axis
	glRotated( th , 0 , 1 , 0 ); // 1. Rotate around the Y axis

	
	draw_cuboid_lined( 0.0f, 0.0f, 0.0f,
                       1.0f, 2.0f, 3.0f,
                       cubClr, linClr );

	//  Display parameters
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	
	Print( "HELLO WORLD!" );

	//  Flush and swap
	glFlush();
	glutSwapBuffers();
}



////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////
double dim    =  40; // Dimension of orthogonal box

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

int main( int argc , char* argv[] ){
	
	// // Calc the initial system before the user changes it
	// lorenz_system_trace( &FUNCTRACE , SIGMA , RHO , BETA , 50 , 0.001 , &N_FUNC );
	
	//  Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	
	//  Request 500 x 500 pixel window
	glutInitWindowSize( 1000 , 750 );
	
	//  Create the window
	glutCreateWindow( "James Watson , HW2" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    //  Request double buffered, true color window 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    // Enable z-testing at the full ranger
	glEnable( GL_DEPTH_TEST );
	glDepthRange( 0.0f , 1.0f );
	
	//  Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );
	
	//  Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );
	
	// //  Tell GLUT to call "special" when an arrow key is pressed
	// glutSpecialFunc( special );
	
	// //  Tell GLUT to call "key" when a key is pressed
	// glutKeyboardFunc( key );
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	// // Free memory
	// matrix_del_f( FUNCTRACE , N_FUNC );
	
	//  Return code
	return 0;
}
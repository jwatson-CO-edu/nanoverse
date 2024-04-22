// g++ Restart.cpp -std=c++17 -O3 -Wall -lm -lglut -lGLU -lGL -o test.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "OGL_Geo.hpp" // _ Utility functions for 5229


////////// STRUCTS /////////////////////////////////////////////////////////////////////////////////




////////// GEOMETRY & DISPLAY DATA /////////////////////////////////////////////////////////////////

///// Geometry /////


///// Display /////
Camera3D camera{};

void load_geo(){
    // Set the scene for display
    
}

////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////

void display(){
	// Display the scene
    /// Init frame ///
	OGL_frame_start();
    ///// Per-Frame Rendering /////////////////////////////////////////////


    ///// Finish Frame ////////////////////////////////////////////////////
	OGL_frame_end();
}


void idle(){
    // Run this when the user isn't doing anything
	
    ///// Idle Bookkeeping ////////////////////////////////////////////////


    ///// Repaint /////////////////////////////////////////////////////////
	glutPostRedisplay();
}


OGL_window win{};

void reshape( int width , int height ){
    // GLUT calls this routine when the window is resized
    // Ratio of the width to the height of the window
    win.w2h = (height > 0) ? (float) width/height : 1;
    // Set the viewport to the entire window
    glViewport( 0, 0, width, height );
    // Set projection
    win.Project();
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main( int argc , char* argv[] ){

    // Initialize GLUT and process user parameters
	glutInit( &argc , argv );

    ///// Create Geometry /////////////////////////////////////////////////


    ///// Setup GLUT && Start Rendering ///////////////////////////////////

    win.create( 500, 500 );

	// Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
	glutIdleFunc( idle );
	
	// Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );

    // Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	// Return code
	return 0;
}
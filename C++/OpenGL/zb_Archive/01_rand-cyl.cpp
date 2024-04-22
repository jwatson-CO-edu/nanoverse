// g++ 01_Rand-cyl.cpp -std=c++17 -O3 -Wall -lm -lglut -lGLU -lGL -o cylRand.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "OGL_Geo.hpp" // _ Utility functions for 5229



////////// STRUCTS /////////////////////////////////////////////////////////////////////////////////

struct CylData{
    // Bookkeeping for one cylinder
    float length;
    float radius; 
    uint  facets;
    vec3f fillClr; 
    vec3f lineClr;
    vec3f trans;
    vec3f rotat;
};



////////// GEOMETRY & DISPLAY DATA /////////////////////////////////////////////////////////////////

///// Geometry /////
vector<CylData> cylinders;
uint /*------*/ N_cyl /**/ =  1;
float /*-----*/ dimLim     =  2.0f; 
float /*-----*/ bbox[2][3] = { {-dimLim, -dimLim, -dimLim}, {dimLim, dimLim, dimLim} };
float /*-----*/ clrSpace[2][3] = { {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f} };
float /*-----*/ rotSpace[2][3] = { {0.0f, 0.0f, 0.0f}, {M_PI, M_PI, M_PI} };
///// Display /////
Camera3D   camera{};
OGL_window win{};

void load_geo(){
    // Set the scene for display
    /// Generate a bunch of cylinders ///
    for( uint i = 0; i < N_cyl; ++i ){
        cylinders.push_back( CylData{
            randf( 0.10f, 1.25f ),
            randf( 0.10f, 1.25f ),
            20,
            sample_from_AABB_f( clrSpace ),
            {0.0f, 0.0f, 0.0f},
            sample_from_AABB_f( bbox ),
            {0.0f, 0.0f, 0.0f} // sample_from_AABB_f( rotSpace )
        } );
        cout << "Constructed cylinder " << (i+1) << endl;
    }
    cout << "There are " << cylinders.size() << " cylinders!" << endl;
    /// Position camera ///
    camera.set_position( {12.0f, 0.0f, 0.0f} );
    camera.set_target(   { 0.0f, 0.0f, 0.0f} );
}


////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////

void display(){
	// Display the scene
    /// Init frame ///
	//  Clear the image
	glClearDepth( 1.0 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
    ///// Per-Frame Rendering /////////////////////////////////////////////

    camera.look();

    /// Draw all cylinders ///
    for( CylData& cyl : cylinders ){
        // push_transform( cyl.trans, cyl.rotat );
        
        // 6. Push
		glPushMatrix();
        
        glRotated( (double) cyl.rotat[0], 1.0, 0.0, 0.0 ); // 1. Rotate around the X axis
        glRotated( (double) cyl.rotat[1], 0.0, 1.0, 0.0 ); // 2. Rotate around the Y axis
        glRotated( (double) cyl.rotat[2], 0.0, 0.0, 1.0 ); // 3. Rotate around the Z axis

        cout << "Transation: " << cyl.trans << ", Rotation: " << cyl.rotat << endl;
        draw_cylinder_lined( {0.0f, 0.0f, 0.0f}, cyl.length, cyl.radius, cyl.facets, cyl.fillClr, cyl.lineClr );

        glTranslated( (double) cyl.trans[0], (double) cyl.trans[1], (double) cyl.trans[2] );

        glPopMatrix();
    }

    ///// Finish Frame ////////////////////////////////////////////////////
	// Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();
}


void idle(){
    // Run this when the user isn't doing anything
	


	// Reproject
	win.Project();
	// Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}




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
    // Program entry point
    init_rand(); // initialize random seed based on system clock

    ///// Create Geometry /////////////////////////////////////////////////
    load_geo();

    ///// Setup GLUT && Start Rendering ///////////////////////////////////

    // Initialize GLUT and process user parameters
	glutInit( &argc , argv );

    //  Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );

    win.create( 1000, 1000 );

    // Don't ask questions!
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0, 1.0 );

	// Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
	glutIdleFunc( idle );
	
	// Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );

    // Check for errors
	cout << "Error Code: " << ErrCheck( "main" ) << endl;

    // Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	// Return code
	return 0;
}
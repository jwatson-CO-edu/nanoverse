/*
MAIN_TEMPLATE.cpp
James Watson , YYYY MONTHNAME
A ONE-LINE DESCRIPTION OF THE FILE

Dependencies:
Template Version: 2018-06-06
*/

/***** DEV PLAN *****
[ ] TODO_i
*/

/********** Init *********************************************************************************/

/***** Includes *****/

/*** Standard ***/

/*** Eigen ***/

/** Special **/

/** Local **/
#include "../helpers/config.hpp"
#include "../helpers/OGL_utils.hpp"
#include "../geometry/geoSketch.cpp"



/***** Constants & Globals *****/


typeF /*--*/ w2h = 0.0; // Aspect ratio
typeF /*--*/ dim = 2.0; // Scale Dimension
int /*----*/ fov = 55; // Field of view (for perspective)
Mesh::Mesh   cube;
Model::Model mCube;

/********** Callbacks ****************************************************************************/

static void Project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	
	//  Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	
	gluPerspective( fov , // -- Field of view angle, in degrees, in the y direction.
					w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					dim/4 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					4*dim ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	
	// Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// Undo previous transformations
	glLoadIdentity();
}

void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
	// Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// Set projection
	Project();
}

/********** Draw Loop ****************************************************************************/

void display(){

    // clear buffer
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

    // Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );

    //  Reset previous transforms to the identity matrix
    glLoadIdentity();

    set_camera( -3.0, -3.0, -3.0, 
                0.0, 0.0, 0.0,
                0.0, 0.0, 1.0 );

    // save the initial ModelView matrix before modifying ModelView matrix
    glPushMatrix();

    // tramsform camera
    // glTranslatef(0, -2.5, 0);
    // glRotatef( 0.5, 1, 0, 0);   // pitch
    // glRotatef(cameraAngleY, 0, 1, 0);   // heading

    mCube.draw();

    // Pop the modified ModelView matrix
    glPopMatrix();

    glFlush();
    // Blit
    glutSwapBuffers();
}


/********** MAIN *********************************************************************************/

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments

    // 1. Init GLUT
    OGL_ContextConfig params = OGL_ContextConfig();

    cout << "Calling `init_GLUT` with result:" << 
         init_GLUT( argc, argv , params , display ) 
         << endl;

    cube  = Mesh::Cuboid();
    mCube = Model::Model( cube, 0 );

    cout << "Cube has " << cube.V.size() << " vertices." << endl;

    set_redraw_functions( reshape );

    // 2. Init OGL
    init_OGL( params );

    default_light_source();

    

    // the last GLUT call (LOOP)
    // window will be shown and display callback is triggered by events
    // NOTE: this call never return main().
    glutMainLoop(); /* Start GLUT event-processing loop */

    // Check for errors
    char* where = nullptr; 
    if( ErrCheck( where ) ){
        cout << "There were errors:" << endl << where << endl << endl;
    }else{
        cout << "There were no errors." << endl;
    }

    return 0; // I guess everything turned out alright at the end!
}




/********** SPARE PARTS ***************************************************************************




*/
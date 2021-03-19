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

// Create a cube in array mode
Mesh::Mesh   cube;
Model::Model mCube;

/********** <CLASS1> *****************************************************************************/



/********** Draw Loop ****************************************************************************/

void display(){

    // clear buffer
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

    set_camera( -1.0, -1.0, -1.0, 
                0.0, 0.0, 0.0,
                0.0, 0.0, 1.0 );

    // save the initial ModelView matrix before modifying ModelView matrix
    // glPushMatrix();

    // tramsform camera
    // glTranslatef(0, -2.5, 0);
    // glRotatef( 0.5, 1, 0, 0);   // pitch
    // glRotatef(cameraAngleY, 0, 1, 0);   // heading

    

    mCube.draw();

    // Pop the modified ModelView matrix
    // glPopMatrix();

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

    set_redraw_functions();

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
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


/********** <CLASS1> *****************************************************************************/



/********** Draw Loop ****************************************************************************/

void display(){
    // Check for errors
    char* where = nullptr; 
    if( ErrCheck( where ) ){
        cout << "There were errors:" << endl << where << endl << endl;
    }else{
        cout << "There were no errors." << endl;
    }
}


/********** MAIN *********************************************************************************/

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments

    // 1. Init GLUT
    OGL_ContextConfig params = OGL_ContextConfig();
    cout << "Calling `init_GLUT` with result:" << 
         init_GLUT( argc, argv , params , display ) 
         << endl;

    // 2. Init OGL
    init_OGL( params );

    // the last GLUT call (LOOP)
    // window will be shown and display callback is triggered by events
    // NOTE: this call never return main().
    glutMainLoop(); /* Start GLUT event-processing loop */

    return 0; // I guess everything turned out alright at the end!
}




/********** SPARE PARTS ***************************************************************************




*/
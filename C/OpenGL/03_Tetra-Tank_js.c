////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _SCALE /**/ =  10.0f; // Scale Dimension
const int   _FOV_DEG    =  55; // - Field of view (for perspective)
const float _TARGET_FPS =  60.0f; // Desired framerate

/// Model Settings ///
const float _TNK_BODY_SCL = 1.0f;
const vec4f _TNK_BODY_CLR = {0.0f, 1.0f, 0.0f, 1.0f};
const float _TNK_WHL_SCL  = 0.25f;
const vec4f _TNK_WHL_CLR  = {0.0f, 0.0f, 1.0f, 1.0f};
const float _GRID_UNIT    =  1.0f;
const uint  _N_UNIT /*-*/ = 50;
const vec4f _GRID_CLR     = {0.5f, 0.5f, 0.5f, 1.0f};
const float _GRID_THICC   = 1.5f;


////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

typedef struct{
    // Tetrahedral vehicle that rolls on 3 icosahedra and translates in any direction on X-Y plane
    VAO_VNC_f* body;
    float*     w0pose; 
    float*     w1pose; 
    float*     w2pose; 
}TetraTank_mk0;


TetraTank_mk0* make_TetraTank_mk0( float bodyRad_m, const vec4f bodyClr, float wheelRad_m, const vec4f wheelClr ){
    // Alloc and construct tank geometry
    TetraTank_mk0* rtnTank = (TetraTank_mk0*) malloc( sizeof( TetraTank_mk0 ) );
    // Body //
    rtnTank->body = tetrahedron_VAO_VNC_f( bodyRad_m, bodyClr ); 
    vec4f v0 /**/ = make_vec4f( rtnTank->body->V[0], rtnTank->body->V[1], rtnTank->body->V[2] );
    vec4f v1 /**/ = make_vec4f( rtnTank->body->V[3], rtnTank->body->V[4], rtnTank->body->V[5] );
    vec4f v2 /**/ = make_vec4f( rtnTank->body->V[6], rtnTank->body->V[7], rtnTank->body->V[8] );
    vec4f segCntr = seg_center( v0, v1 );
    float rotAngl = angle_between_vec4f( sub_vec4f( v2, segCntr ), make_vec4f( 0.0f, 0.0f, 1.0f ) );
    float op2[16];
    translate_mtx44f( rtnTank->body->relPose, 0.0f, 0.0f, bodyRad_m );
    Rx_mtx44f( op2, -(M_PI/2.0f-rotAngl) );
    mult_mtx44f( rtnTank->body->relPose, op2 );
    // Wheels //
    allocate_N_VAO_VNC_parts( rtnTank->body, 3 );
    for( ubyte i = 0; i < 3; ++i ){
        rtnTank->body->parts[i] = (void*) icosahedron_VAO_VNC_f( wheelRad_m, wheelClr );
    }
    rtnTank->w0pose = get_part_i( rtnTank->body, 0 )->ownPose; 
    rtnTank->w1pose = get_part_i( rtnTank->body, 1 )->ownPose; 
    rtnTank->w2pose = get_part_i( rtnTank->body, 2 )->ownPose;
    v0 = stretch_to_len_vec4f( v0, bodyRad_m + wheelRad_m*2.0f );
    v1 = stretch_to_len_vec4f( v1, bodyRad_m + wheelRad_m*2.0f );
    v2 = stretch_to_len_vec4f( v2, bodyRad_m + wheelRad_m*2.0f );
    translate_mtx44f( get_part_i( rtnTank->body, 0 )->relPose, v0.x, v0.y, v0.z );
    translate_mtx44f( get_part_i( rtnTank->body, 1 )->relPose, v1.x, v1.y, v1.z );
    translate_mtx44f( get_part_i( rtnTank->body, 2 )->relPose, v2.x, v2.y, v2.z );
    // Alloc All @ GPU //
    allocate_and_load_VAO_VNC_at_GPU( rtnTank->body );
    // Return //
    return rtnTank;
}

////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////
TetraTank_mk0* tank = NULL;
Camera3D /*-*/ cam  = { {4.0f, 2.0f, 2.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f} };



////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    
    

    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    //  Erase the window and the depth buffer
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glLoadIdentity();

    ///// DRAW LOOP BEGIN /////////////////////////////////////////////////
    look( cam );

    draw_grid_org_XY( _GRID_UNIT, _N_UNIT, _N_UNIT, 
                      _GRID_THICC, _GRID_CLR );

    draw_VAO_VNC_f( tank->body );

    ///// DRAW LOOP END ///////////////////////////////////////////////////

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
    glColor3f( 1.0f, 1.0f, 1.0f );
    Print( "FPS=%f", heartbeat_FPS( _TARGET_FPS ) );

    // Check for errors, Flush, and swap
	ErrCheck( "display" );
	glFlush();
	glutSwapBuffers();
}


////////// WINDOW & VIEW STATE /////////////////////////////////////////////////////////////////////
float w2h = 0.0f; // Aspect ratio


static void project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	// 1. Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	gluPerspective( _FOV_DEG , //- Field of view angle, in degrees, in the y direction.
					w2h , // ----- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					_SCALE/4 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					4*_SCALE ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	// 2. Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// 3. Undo previous transformations
	glLoadIdentity();
}


void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// 1. Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// 2. Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// 3. Set projection
	project();
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){

    init_rand();

    ///// Initialize GLUT /////////////////////////////////////////////////

    glutInit( &argc , argv );
    // initGL();

    // Request window with size specified in pixels
    glutInitWindowSize( 900, 900 );

    // Create the window
    glutCreateWindow( "Vertex Array Object (VAO) Test" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0f , 1.0f ); // WARNING: NOT IN THE EXAMPLE


    ///// Initialize Geometry /////////////////////////////////////////////
    tank = make_TetraTank_mk0( _TNK_BODY_SCL, _TNK_BODY_CLR, _TNK_WHL_SCL, _TNK_WHL_CLR );
    

    ///// Initialize GLUT Callbacks ///////////////////////////////////////

    //  Tell GLUT to call "display" when the scene should be drawn
    glutDisplayFunc( display );

    // Tell GLUT to call "idle" when there is nothing else to do
    glutIdleFunc( tick );
    
    //  Tell GLUT to call "reshape" when the window is resized
    glutReshapeFunc( reshape );
    
    // //  Tell GLUT to call "special" when an arrow key is pressed
    // glutSpecialFunc( special );
    
    // //  Tell GLUT to call "key" when a key is pressed
    // glutKeyboardFunc( key );


    ///// GO ///// GO ///// GO ////////////////////////////////////////////
    
    // Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    
    ///// Free Memory /////////////////////////////////////////////////////
    printf( "Cleanup!\n" );
    
    delete_VAO_VNC_f( tank->body );

    printf( "\n### DONE ###\n\n" );
    //  Return code
    return 0;
}
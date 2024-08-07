// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _SCALE /**/ =  10.0f; // Scale Dimension
const int   _FOV_DEG    =  55; // - Field of view (for perspective)
const float _TARGET_FPS =  60.0f; // Desired framerate

/// Animation Settings ///
const float _DEL_THETA_RAD = M_PI/90.0f;
const vec4f _ROT_AXIS /**/ = { 1.0f, 1.0f, 1.0f, 1.0f};
const vec4f _SUB_AXIS /**/ = {-1.0f, 1.0f, 1.0f, 1.0f};



////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////
VNCT_f*  cube = NULL;
Camera3D cam  = { {4.0f, 2.0f, 2.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f} };



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

void fetch_pose( float mat16[], float* dataStore ){
    // Copy the 4x4 matrix from the allocated memory block beginning at `dataStore`
    float* fltPtr = dataStore;
    for( ubyte i = 0; i < 16; ++i ){
        mat16[i] = *dataStore;
        ++dataStore;
    }
}


void store_pose( float* dataStore, float mat16[] ){
    // Store the 4x4 matrix in the allocated memory block beginning at `dataStore`
    float* fltPtr = dataStore;
    for( ubyte i = 0; i < 16; ++i ){
        *dataStore = mat16[i];
        ++dataStore;
    }
}


static const float I_33f[] = {1,0,0 , 0,1,0 , 0,0,1};


void get_rot_matx_from_homog( float rot33f[], float mat16f[] ){
    // Obtain the rotation matrix from the homogeneous transform
    rot33f[0] = mat16f[0];  rot33f[3] = mat16f[4];  rot33f[6] = mat16f[ 8];  
    rot33f[1] = mat16f[1];  rot33f[4] = mat16f[5];  rot33f[7] = mat16f[ 9];  
    rot33f[2] = mat16f[2];  rot33f[5] = mat16f[6];  rot33f[8] = mat16f[10];  
}


void transpose_mtx33f( float mat9f[] ){
    // Transpose the 3x3 matrix
    float res[9];
    res[0] = mat9f[0];  res[3] = mat9f[1];  res[6] = mat9f[2];  
    res[1] = mat9f[3];  res[4] = mat9f[4];  res[7] = mat9f[5];  
    res[2] = mat9f[6];  res[5] = mat9f[7];  res[8] = mat9f[8]; 
    memcpy( mat9f, res, sizeof( res ) );
}

void mult_mtx33f( float mat[], float m[] ){
    // Right multiply 3x3 matrix
    float res[9];
    for( int i = 0; i < 3; i++ )
        for( int j = 0; j < 3; j++ )
            res[ 3*i+j ] = mat[j]*m[3*i] + mat[3+j]*m[3*i+1] + mat[6+j]*m[3*i+2];
    //  Copy matrix back
    memcpy( mat, res, sizeof( res ) );
}


void sub_mtx33f( float mat[], float m[] ){
    // Subtract: `mat` - `m`
    for( int i = 0; i < 9; i++ ){  mat[i] -= m[i];  }
}


float norm_mtx33f( float mat[] ){
    // Return the sum of all elements
    float tot = 0.0f;
    for( int i = 0; i < 9; i++ ){  tot += mat[i];  }
    return tot;
}


void identity_mtx33f( float mat[] ){  memcpy( mat, I_33f, sizeof( I_33f ) );  } // Identity 3x3 matrix


void copy_mtx33f( float mat[], const float m[] ){
    // Copy 3x3 matrix
    memcpy( mat, m, sizeof( I_33f ) );
}


ubyte p_rotation_mtrx( float R[] ){
    // Checks if a matrix is a valid rotation matrix.
    float Rt[9];
    float df;
    copy_mtx33f( Rt, R );
    transpose_mtx33f( Rt );
    mult_mtx33f( Rt, R );
    sub_mtx33f( Rt, I_33f );
    df = norm_mtx33f( Rt );
    if( df < 1e-5 )  return 1;
    else /*------*/  return 0;
}
    



////////// AGENT LOGIC /////////////////////////////////////////////////////////////////////////////

///// VBO Keyframe Animation //////////////////////////////////////////////

/* /// MoveVNCT Data Layout ///
refs  [0]    : VBO Model ------------------------------------- (Set by factory function)
state [ 0-15]: Target pose ----------------------------------- (Set by factory function)
state [16-31]: Update transform ------------------------------ (Set by init    function)
state [32]   : Desired linear speed -------------------------- (Set by factory function)
state [33]   : Flag for relative move, <= 0.0 is absolute move (Set by factory function)
*/

BT_Pckt MoveVNCT_init( void* behav, BT_Pckt input ){
    // Compute the update transform for this keyframe animation
    VNCT_f* model = (VNCT_f*) (((Behavior*) behav)->refs[0]);
    float   target[16]; // - Where we are going?
    float   bgnXfrm[16]; //- Where did we start?
    float   xfrmStep[16]; // How to get there
    vec4f   bgnPosn, endPosn;
    ubyte   relMove = (((Behavior*) behav)->state[33] > 0.0);
    fetch_pose( target, (((Behavior*) behav)->state) );
    
    if( relMove ){
        copy_mtx44f( bgnXfrm, model->ownPose );
        // FIXME: RELATIVE CALCS
    }else{
        update_total_pose( model );
        copy_mtx44f( bgnXfrm, model->totPose );
        // FIXME: ABSOLUTE CALCS
    }

    // FIXME, START HERE: COMPUTE THE PER-TICK UPDATE TRANSFORM

}


BT_Pckt MoveVNCT_update( void* behav, BT_Pckt input ){
    VNCT_f* model = (VNCT_f*) (((Behavior*) behav)->refs[0]);
}

// FIXME: WRITE THE FACTORY FUNCTION FOR THIS LEAF NODE


////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    
    rotate_angle_axis_rad( cube, _DEL_THETA_RAD, _ROT_AXIS );

    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    // vec4f center = {0.0f,0.0f,0.0f,1.0f};
    // vec4f sphClr = {0.0, 14.0f/255.0f, (214.0f-75.0f)/255.0f,1.0f};

    //  Erase the window and the depth buffer
    // glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable( GL_DEPTH_TEST );
    glLoadIdentity();

    look( cam );
    
    draw_VNT_f( cube );

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
    printf( "About to init context ...\n" );
    init_rand();
    // Initialize GLUT and process user parameters
    glutInit( &argc , argv );
    // initGL();

    // Request window with size specified in pixels
    glutInitWindowSize( 900, 900 );

    // Create the window
    glutCreateWindow( "Vertex Array Object (VBO) Test" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0f , 1.0f ); // WARNING: NOT IN THE EXAMPLE

    printf( "About to init cube ...\n" );
    cube = cube_VNT_f();
    set_texture( cube, "resources/crate.bmp" );
    allocate_and_load_VBO_VNT_at_GPU( cube );
    allocate_N_VBO_VNCT_parts( cube, 1 );

     
    

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
    
    // Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    // Free memory

    printf( "Cleanup!\n" );
    delete_VNCT_f( cube );

    printf( "\n### DONE ###\n\n" );
    
    //  Return code
    return 0;
}
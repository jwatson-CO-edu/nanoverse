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


void get_rot_matx_from_homog( float rot9f[], float mat16f[] ){
    // Obtain the rotation matrix from the homogeneous transform
    rot9f[0] = mat16f[0];  rot9f[3] = mat16f[4];  rot9f[6] = mat16f[ 8];  
    rot9f[1] = mat16f[1];  rot9f[4] = mat16f[5];  rot9f[7] = mat16f[ 9];  
    rot9f[2] = mat16f[2];  rot9f[5] = mat16f[6];  rot9f[8] = mat16f[10];  
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
    // Checks if a 3x3 matrix is a valid rotation matrix.
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
    

ubyte p_homog_rotation_OK( float mat16f[] ){
    // Checks if a transform has a valid rotation matrix.
    float rot[9];
    get_rot_matx_from_homog( rot, mat16f );
    return p_rotation_mtrx( rot );
}

void pose_mtrx_to_vec( float vec6f[], float mat16f[] ){
    float theta, rv1, rv2, rv3, xx, yy, zz, xy, xz, yz, x, y, z, r11, r12, r13, r21, r22, r23, r31, r32, r33;
    if( p_homog_rotation_OK( mat16f ) ){

        // ##### Position: The Easy Part ################
        x = mat16f[3*4 + 0];
        y = mat16f[3*4 + 1];
        z = mat16f[3*4 + 2];

        // ##### Orientation: The Tricky Part ################
        r11 = mat16f[0 + 0*4];
        r12 = mat16f[0 + 1*4];
        r13 = mat16f[0 + 2*4];
        r21 = mat16f[1 + 0*4];
        r22 = mat16f[1 + 1*4];
        r23 = mat16f[1 + 2*4];
        r31 = mat16f[2 + 0*4];
        r32 = mat16f[2 + 1*4];
        r33 = mat16f[2 + 2*4];

        // NOTE: THIS IS TO PREVENT FLIPPING FOR HAND-CODED AXIS-ALSIGNED POSES, WHICH SEEMS TO BE COMMON
        // NOTE: STOP REMOVING THIS PART, **I WILL FIND OUT**
        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
        float epsilon  = 1e-4; // margin to allow for rounding errors
        float epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees

        if((fabsf(r12-r21) < epsilon) && (fabsf(r13-r31) < epsilon) && (fabsf(r23-r32)< epsilon)){
            // Singularity found, First check for identity matrix which must have +1 for all terms, in leading diagonal and zero in other terms
            if((fabsf(r12+r21) < epsilon2) && (fabsf(r13+r31) < epsilon2) && (fabsf(r23+r32) < epsilon2) && (fabsf(r11+r22+r33-3.0) < epsilon2)):
        }

    }else{
        for( ubyte i = 0; i < 6; ++i ){  vec6f[i] = 0.0f;  }
    }
}
    
    
    

    

    

    
        
        
			# this singularity is identity matrix so angle = 0
            theta = 0.0
            rv1   = 0.0
            rv2   = 0.0
            rv3   = 0.0
        else:
            theta = np.pi
            xx = (r11 + 1.0) / 2.0
            yy = (r22 + 1.0) / 2.0
            zz = (r33 + 1.0) / 2.0
            xy = (r12 + r21) / 4.0
            xz = (r13 + r31) / 4.0
            yz = (r23 + r32) / 4.0
            if ((xx > yy) and (xx > zz)): # m[0][0] is the largest diagonal term
                if (xx < epsilon):
                    kx = 0.0
                    ky = 0.7071
                    kz = 0.7071
                else:
                    kx = np.sqrt( xx )
                    ky = xy / kx
                    kz = xz / kx
            elif (yy > zz): # m[1][1] is the largest diagonal term
                if (yy < epsilon):
                    kx = 0.7071
                    ky = 0.0
                    kz = 0.7071
                else:
                    ky = np.sqrt( yy )
                    kx = xy / ky
                    kz = yz / ky
            else: # m[2][2] is the largest diagonal term so base result on this
                if (zz < epsilon):
                    kx = 0.7071
                    ky = 0.7071
                    kz = 0.0
                else:
                    kz = np.sqrt( zz )
                    kx = xz / kz
                    ky = yz / kz

		
    else:
        val = (r11 + r22 + r33 - 1) / 2.0
        while val < -1.0:
            val += 2.0
        while val > 1.0:
            val -= 2.0
        theta = np.arccos( val )

        if theta == 0.0:
            theta = 1e-8
        sth = np.sin(theta)
        kx = (r32 - r23) / (2 * sth)
        ky = (r13 - r31) / (2 * sth)
        kz = (r21 - r12) / (2 * sth)

    rv1 = theta * kx
    rv2 = theta * ky
    rv3 = theta * kz

    

    return [float(x), float(y), float(z), float(rv1), float(rv2), float(rv3)]


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
// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"
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


////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

typedef struct{
    // Vertex Array Object meant to be drawn rapidly and simply

    /// Geo Info ///
    uint   Ntri; //- Number of triangles
    float* V; // --- `Ntri` * 9: `float`
    float* N; // --- `Ntri` * 9: `float`
    float* C; // --- `Ntri` * 9: `float`
    uint   bufID; // Buffer ID at the GPU
    uint   arSiz; // Array size in bytes

    /// Texture Info ///
    float* T; // --- `Ntri` * 9: `float`
    uint   txSiz; // UV Array size in bytes
    uint   texID; // Handle for the texture
    
    /// Pose & Scale ///
    float* relPose; // Static offset pose from parent pose (anchor)
    float* ownPose; // Dynamic offset pose from `relPose`
    vec4f  scale; // - Scale in each dimension
    float* totPose; // Total pose relative to parent frame, Accounting for {`relPose`, `ownPose`, `scale`}
    
    /// Composite VAO ///
    uint   Nprt; //- Number of sub-parts
    void** parts; // Array of sub-part pointers

}VNCT_f;


VNCT_f* make_VNCT_f( uint Ntri_ ){
    // Allocate the VAO at heap
    uint    arrSize = sizeof( float ) * 9 * Ntri_;
    uint    texSize = sizeof( float ) * 6 * Ntri_;
    VNCT_f* rtnVAO  = (VNCT_f*) malloc( sizeof( VNCT_f ) );
    /// Geo Info ///
    rtnVAO->Ntri  = Ntri_;
    rtnVAO->V     = (float*) malloc( arrSize );
    rtnVAO->N     = (float*) malloc( arrSize );
    rtnVAO->C     = (float*) malloc( arrSize );
    rtnVAO->bufID = 0;
    rtnVAO->arSiz = arrSize;
    /// Texture Info ///
    rtnVAO->T     = (float*) malloc( texSize );
    rtnVAO->txSiz = texSize;
    rtnVAO->texID = 0;
    /// Pose & Scale ///
    rtnVAO->relPose = make_identity_mtx44f();
    rtnVAO->ownPose = make_identity_mtx44f();
    rtnVAO->scale   = make_vec4f( 1.0f, 1.0f, 1.0f );
    rtnVAO->totPose = make_identity_mtx44f();
    /// Composite VAO ///
    rtnVAO->Nprt  = 0;
    rtnVAO->parts = NULL;
    /// Return ///
    return rtnVAO;
}


void allocate_N_VAO_VNCT_parts( VNCT_f* vao, uint N ){
    // Make space for `N` sub-part pointers
    vao->Nprt  = N;
    vao->parts = (void**) malloc( N * sizeof( VNCT_f* ) );
}


void set_texture( VNCT_f* vao, const char* path ){
    // Load texture at GPU and get its handle
    vao->texID = LoadTexBMP( path );
}


// VNCT_f* get_part_i( VNCT_f* vao, uint i ){
//     // Fetch sub-VAO, NOTE: This is to wrap the need to cast the null pointer
//     if( i < vao->Nprt )  return (VNCT_f*) vao->parts[i];
//     else /*----------*/  return NULL;
// }


void delete_VNCT_f( VNCT_f* vao ){
    // Erase the VAO (and parts) at heap and the GPU
    if((vao->bufID) != 0){  glDeleteBuffersARB(1, &(vao->bufID));  }
    free( vao->V );
    free( vao->N );
    free( vao->C );
    free( vao->T );
    free( vao->relPose );
    free( vao->ownPose );
    for( uint i = 0; i < vao->Nprt; ++i ){   delete_VNCT_f( (VNCT_f*) (vao->parts[i]) );  }
    free( vao );
}


void load_VNC_from_full_arrays( VNCT_f* vao, /*<<*/ const float* Vsto, const float* Nsto, const float* Csto ){
    // Copy {V,N,C} from the specified arrays
    memcpy( vao->V, Vsto, vao->arSiz ); // Copy vertices
    memcpy( vao->N, Nsto, vao->arSiz ); // Copy normals
    memcpy( vao->C, Csto, vao->arSiz ); // Copy colors
}


void load_VNT_from_full_arrays( VNCT_f* vao, /*<<*/ const float* Vsto, const float* Nsto, const float* Tsto ){
    // Copy {V,N,C} from the specified arrays
    memcpy( vao->V, Vsto, vao->arSiz ); // Copy vertices
    memcpy( vao->N, Nsto, vao->arSiz ); // Copy normals
    memcpy( vao->T, Tsto, vao->txSiz ); // Copy texture UV
}


void allocate_and_load_VAO_VNC_at_GPU( VNCT_f* vao ){
    // Fetch & set buffer ID, and make space on the GPU for the VAO
    glGenBuffersARB( 1, &(vao->bufID) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, 3*(vao->arSiz), 0, GL_STATIC_DRAW_ARB );
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 0             , vao->arSiz, vao->V ); // copy vertices starting from 0 offest
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, vao->arSiz    , vao->arSiz, vao->N ); // copy normals after vertices
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 2*(vao->arSiz), vao->arSiz, vao->C ); // copy colours after normals
}


void allocate_and_load_VAO_VNT_at_GPU( VNCT_f* vao ){
    // Fetch & set buffer ID, and make space on the GPU for the VAO
    glGenBuffersARB( 1, &(vao->bufID) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, 3*(vao->arSiz), 0, GL_STATIC_DRAW_ARB );
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 0             , vao->arSiz, vao->V ); // copy vertices starting from 0 offest
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, vao->arSiz    , vao->arSiz, vao->N ); // copy normals after vertices
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 2*(vao->arSiz), vao->txSiz, vao->T ); // copy UV after normals
}


void update_total_pose( VNCT_f* vao ){
    // Compose relative, ownship, and scale transformations into `totPose`, relative to parent frame
    identity_mtx44f( vao->totPose );
    mult_mtx44f( vao->totPose, vao->relPose );
    mult_mtx44f( vao->totPose, vao->ownPose );
    scale_mtx44f( vao->totPose, vao->scale.x, vao->scale.y, vao->scale.z );
}


void draw_recur_VNC_f( VNCT_f* vao ){
    // Inner draw function for `VNCT_f`, NOTE: This function assumes GPU is set to draw VAO
    ulong arrSize = vao->arSiz;
    ulong dblSize = arrSize*2;

    update_total_pose( vao );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( vao->totPose );
    
    // bind VBOs with IDs and set the buffer offsets of the bound VBOs
    // When buffer object is bound with its ID, all pointers in gl*Pointer()
    // are treated as offset instead of real pointer.
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );

    // before draw, specify vertex and index arrays with their offsets
    glVertexPointer( 3, GL_FLOAT, 0, 0               );
    glNormalPointer(    GL_FLOAT, 0, (void*) arrSize );
    glColorPointer(  3, GL_FLOAT, 0, (void*) dblSize );

    glDrawArrays( GL_TRIANGLES, 0, 3*(vao->Ntri) );

    // it is good idea to release VBOs with ID 0 after use.
    // Once bound with 0, all pointers in gl*Pointer() behave as real
    // pointer, so, normal vertex array operations are re-activated
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

    for( uint i = 0; i < vao->Nprt; ++i ){  draw_recur_VNC_f( (VNCT_f*) vao->parts[i] );  }

    glPopMatrix();
}


void draw_VNC_f( VNCT_f* vao ){
    // Draw using "VBO Method" (See Song Ho Ahn code)
    // printf( "About to draw VAO ...\n" );
    
    // enable vertex arrays
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_NORMAL_ARRAY );
    glEnableClientState( GL_COLOR_ARRAY  );

    draw_recur_VNC_f( vao );

    // disable vertex arrays
    glDisableClientState( GL_VERTEX_ARRAY );  
    glDisableClientState( GL_NORMAL_ARRAY );
    glDisableClientState( GL_COLOR_ARRAY  );
}


void draw_recur_VNT_f( VNCT_f* vao ){
    // Inner draw function for `VNCT_f`, NOTE: This function assumes GPU is set to draw VAO
    ulong arrSize = vao->arSiz;
    ulong dblSize = arrSize*2;

    update_total_pose( vao );

    glColor4f( 1,1,1,1 ); // Set all color channels to full
    glEnable( GL_TEXTURE_2D );
    glBindTexture( GL_TEXTURE_2D , vao->texID );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glMultMatrixf( vao->totPose );
    
    // bind VBOs with IDs and set the buffer offsets of the bound VBOs
    // When buffer object is bound with its ID, all pointers in gl*Pointer()
    // are treated as offset instead of real pointer.
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );

    // before draw, specify vertex and index arrays with their offsets
    glVertexPointer(   3, GL_FLOAT, 0, 0               );
    glNormalPointer(      GL_FLOAT, 0, (void*) arrSize );
    glTexCoordPointer( 2, GL_FLOAT, 0, (void*) dblSize );

    glDrawArrays( GL_TRIANGLES, 0, 3*(vao->Ntri) );

    // it is good idea to release VBOs with ID 0 after use.
    // Once bound with 0, all pointers in gl*Pointer() behave as real
    // pointer, so, normal vertex array operations are re-activated
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );

    for( uint i = 0; i < vao->Nprt; ++i ){  draw_recur_VNT_f( (VNCT_f*) vao->parts[i] );  }

    glPopMatrix();
}


void draw_VNT_f( VNCT_f* vao ){
    // Draw using "VBO Method" (See Song Ho Ahn code)
    // printf( "About to draw VAO ...\n" );
    
    // enable vertex arrays
    glEnableClientState( GL_VERTEX_ARRAY        );
    glEnableClientState( GL_NORMAL_ARRAY        );
    glEnableClientState( GL_TEXTURE_COORD_ARRAY );

    draw_recur_VNT_f( vao );

    // disable vertex arrays
    glDisableClientState( GL_VERTEX_ARRAY        );  
    glDisableClientState( GL_NORMAL_ARRAY        );
    glDisableClientState( GL_TEXTURE_COORD_ARRAY );
}


vec4f get_posn( VNCT_f* vao ){
    // Get the position components of the homogeneous coordinates as a vector
    return make_vec4f( vao->totPose[12], vao->totPose[13], vao->totPose[14] );
}


void set_posn( VNCT_f* vao, const vec4f posn ){
    // Set the position components of the homogeneous coordinates
    vao->ownPose[12] = posn.x;
    vao->ownPose[13] = posn.y;
    vao->ownPose[14] = posn.z;
}


void translate( VNCT_f* vao, const vec4f delta ){
    // Increment the position components of the homogeneous coordinates by the associated `delta` components
    vao->ownPose[12] += delta.x;
    vao->ownPose[13] += delta.y;
    vao->ownPose[14] += delta.z;
}


void rotate_angle_axis_rad( VNCT_f* vao, float angle_rad, const vec4f axis ){
    // Rotate the object by `angle_rad` about `axis`
    float op1[16];  identity_mtx44f( op1 );
    rotate_angle_axis_mtx44f( op1, angle_rad*180/M_PI, axis.x, axis.y, axis.z );
    // print_mtx44f( "Rotation Matrix:", op1 );
    mult_mtx44f( op1, vao->ownPose );
    copy_mtx44f( vao->ownPose, op1 );
}


void rotate_RPY_vehicle( VNCT_f* vao, float r_, float p_, float y_ ){
    // Increment the world Roll, Pitch, Yaw of the model
    // NOTE: This is for airplanes that move forward in their own Z and have a wingspan across X
    float op1[16];  identity_mtx44f( op1 );
    R_RPY_vehicle_mtx44f( op1, r_, p_, y_ );
    mult_mtx44f( op1, vao->ownPose );
    copy_mtx44f( vao->ownPose, op1 );
}


// void thrust_Z_vehicle( VNCT_f* vao, float dZ ){
//     // Move in the local Z direction by `dZ` 
//     vec4f disp = make_vec4f( 0.0f, 0.0f, dZ );
//     disp = mult_mtx44f_vec4f( vao->ownPose, disp );
//     translate( vao, disp );
// }





///// cube ////////////////////////////////////////////////////////////////
//    v6----- v5
//   /|      /|
//  v1------v0|
//  | |     | |
//  | |v7---|-|v4
//  |/      |/
//  v2------v3

// vertex coords array for glDrawArrays() =====================================
// A cube has 6 sides and each side has 2 triangles, therefore, a cube consists
// of 36 vertices (6 sides * 2 tris * 3 vertices = 36 vertices). And, each
// vertex is 3 components (x,y,z) of floats, therefore, the size of vertex
// array is 108 floats (36 * 3 = 108).
const float cubeV[]  = { 1, 1, 1,  -1, 1, 1,  -1,-1, 1,      // v0-v1-v2 (front)
                        -1,-1, 1,   1,-1, 1,   1, 1, 1,      // v2-v3-v0

                         1, 1, 1,   1,-1, 1,   1,-1,-1,      // v0-v3-v4 (right)
                         1,-1,-1,   1, 1,-1,   1, 1, 1,      // v4-v5-v0

                         1, 1, 1,   1, 1,-1,  -1, 1,-1,      // v0-v5-v6 (top)
                        -1, 1,-1,  -1, 1, 1,   1, 1, 1,      // v6-v1-v0

                        -1, 1, 1,  -1, 1,-1,  -1,-1,-1,      // v1-v6-v7 (left)
                        -1,-1,-1,  -1,-1, 1,  -1, 1, 1,      // v7-v2-v1

                        -1,-1,-1,   1,-1,-1,   1,-1, 1,      // v7-v4-v3 (bottom)
                         1,-1, 1,  -1,-1, 1,  -1,-1,-1,      // v3-v2-v7

                         1,-1,-1,  -1,-1,-1,  -1, 1,-1,      // v4-v7-v6 (back)
                        -1, 1,-1,   1, 1,-1,   1,-1,-1 };    // v6-v5-v4

// normal array
const float cubeN[] = { 0, 0, 1,   0, 0, 1,   0, 0, 1,      // v0-v1-v2 (front)
                        0, 0, 1,   0, 0, 1,   0, 0, 1,      // v2-v3-v0

                        1, 0, 0,   1, 0, 0,   1, 0, 0,      // v0-v3-v4 (right)
                        1, 0, 0,   1, 0, 0,   1, 0, 0,      // v4-v5-v0

                        0, 1, 0,   0, 1, 0,   0, 1, 0,      // v0-v5-v6 (top)
                        0, 1, 0,   0, 1, 0,   0, 1, 0,      // v6-v1-v0

                       -1, 0, 0,  -1, 0, 0,  -1, 0, 0,      // v1-v6-v7 (left)
                       -1, 0, 0,  -1, 0, 0,  -1, 0, 0,      // v7-v2-v1

                        0,-1, 0,   0,-1, 0,   0,-1, 0,      // v7-v4-v3 (bottom)
                        0,-1, 0,   0,-1, 0,   0,-1, 0,      // v3-v2-v7

                        0, 0,-1,   0, 0,-1,   0, 0,-1,      // v4-v7-v6 (back)
                        0, 0,-1,   0, 0,-1,   0, 0,-1 };    // v6-v5-v4

// color array
const float cubeC[] = { 1, 1, 1,   1, 1, 0,   1, 0, 0,      // v0-v1-v2 (front)
                        1, 0, 0,   1, 0, 1,   1, 1, 1,      // v2-v3-v0

                        1, 1, 1,   1, 0, 1,   0, 0, 1,      // v0-v3-v4 (right)
                        0, 0, 1,   0, 1, 1,   1, 1, 1,      // v4-v5-v0

                        1, 1, 1,   0, 1, 1,   0, 1, 0,      // v0-v5-v6 (top)
                        0, 1, 0,   1, 1, 0,   1, 1, 1,      // v6-v1-v0

                        1, 1, 0,   0, 1, 0,   0, 0, 0,      // v1-v6-v7 (left)
                        0, 0, 0,   1, 0, 0,   1, 1, 0,      // v7-v2-v1

                        0, 0, 0,   0, 0, 1,   1, 0, 1,      // v7-v4-v3 (bottom)
                        1, 0, 1,   1, 0, 0,   0, 0, 0,      // v3-v2-v7

                        0, 0, 1,   0, 0, 0,   0, 1, 0,      // v4-v7-v6 (back)
                        0, 1, 0,   0, 1, 1,   0, 0, 1 };    // v6-v5-v4

// color array
const float cubeT[] = { 0,0,   0,1,   1,1,      // v0-v1-v2 (front)
                        1,1,   1,0,   0,0,      // v2-v3-v0

                        0,0,   0,1,   1,1,      // v0-v1-v2 (front)
                        1,1,   1,0,   0,0,

                        0,0,   0,1,   1,1,      // v0-v1-v2 (front)
                        1,1,   1,0,   0,0,

                        0,0,   0,1,   1,1,      // v0-v1-v2 (front)
                        1,1,   1,0,   0,0,

                        0,0,   0,1,   1,1,      // v0-v1-v2 (front)
                        1,1,   1,0,   0,0,

                        0,0,   0,1,   1,1,      // v0-v1-v2 (front)
                        1,1,   1,0,   0,0 };    // v6-v5-v4

VNCT_f* cube_VNT_f( void ){
    // Make a colorful cube from the static array data
    printf( "About to allocate cube ...\n" );
    VNCT_f* rtnVAO = make_VNCT_f( 12 );
    printf( "About to populate cube ...\n" );
    load_VNT_from_full_arrays( rtnVAO, cubeV, cubeN, cubeT );
    return rtnVAO;
}



////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////
VNCT_f*  cube = NULL;
VNCT_f*  sub0 = NULL;
Camera3D cam  = { {4.0f, 2.0f, 2.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f} };



////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    
    rotate_angle_axis_rad( cube, _DEL_THETA_RAD, _ROT_AXIS );
    rotate_angle_axis_rad( sub0, _DEL_THETA_RAD, _SUB_AXIS );

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
    glutCreateWindow( "Vertex Array Object (VAO) Test" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0f , 1.0f ); // WARNING: NOT IN THE EXAMPLE

    printf( "About to init cube ...\n" );
    cube = cube_VNT_f();
    set_texture( cube, "resources/crate.bmp" );
    allocate_and_load_VAO_VNT_at_GPU( cube );
    allocate_N_VAO_VNCT_parts( cube, 1 );

    printf( "About to init subpart ...\n" );
    sub0 = cube_VNT_f();
    set_texture( sub0, "resources/crate.bmp" );
    allocate_and_load_VAO_VNT_at_GPU( sub0 );
    sub0->scale   = make_vec4f( 0.25f, 0.25f, 0.25f );
    translate_mtx44f( sub0->relPose, 1.0f, 1.0f, 1.0f );
    cube->parts[0] = (void*) sub0;
     
    

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
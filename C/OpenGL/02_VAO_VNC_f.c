// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _SCALE /**/ =   5.0; // Scale Dimension
const int   _FOV_DEG    =  55; // - Field of view (for perspective)
const float _TARGET_FPS =  60.0f; // Desired framerate



////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

typedef struct{
    // Vertex Array Object meant to be drawn rapidly and simply
    uint   Nvtx; //- Number of vertices
    float* V; // --- `Nvtx` * 3: `float`
    float* N; // --- `Nvtx` * 3: `float`
    float* C; // --- `Nvtx` * 3: `float`
    uint   bufID; // Buffer ID at the GPU
    // FIXME: SUBPARTS AT A RELATIVE FRAME OF REF
}VAO_VNC_f;


VAO_VNC_f* make_VAO_VNC_f( uint Nvtx_ ){
    // Allocate the VAO at heap
    uint arrSize = sizeof( float ) * 9 * Nvtx_;
    VAO_VNC_f* rtnVAO = (VAO_VNC_f*) malloc( sizeof( VAO_VNC_f ) );
    rtnVAO->Nvtx  = Nvtx_;
    rtnVAO->V     = (float*) malloc( arrSize );
    rtnVAO->N     = (float*) malloc( arrSize );
    rtnVAO->C     = (float*) malloc( arrSize );
    rtnVAO->bufID = 0;
    return rtnVAO;
}


void delete_VAO_VNC_f( VAO_VNC_f* vao ){
    // Erase the VAO at heap and the GPU
    if((vao->bufID) != 0){  glDeleteBuffersARB(1, &(vao->bufID));  }
    free( vao->V );
    free( vao->N );
    free( vao->C );
    free( vao );
}


void load_VAO_VNC_from_full_arrays( VAO_VNC_f* vao, /*<<*/ const float* Vsto, const float* Nsto, const float* Csto ){
    // Copy {V,N,C} from the specified arrays
    uint arrSize = sizeof( float ) * 9 * vao->Nvtx;
    printf( "About to copy vertices ...\n" );
    memcpy( vao->V, Vsto, arrSize ); // Copy vertices
    printf( "About to copy normals ...\n" );
    memcpy( vao->N, Nsto, arrSize ); // Copy normals
    printf( "About to copy colors ...\n" );
    memcpy( vao->C, Csto, arrSize ); // Copy colors
}


void allocate_and_load_VAO_VNC_at_GPU( VAO_VNC_f* vao ){
    // Fetch & set buffer ID, and make space on the GPU for the VAO
    uint arrSize = sizeof( float ) * 9 * (vao->Nvtx);
    glGenBuffersARB( 1, &(vao->bufID) );
    glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );
    glBufferDataARB( GL_ARRAY_BUFFER_ARB, 3*arrSize, 0, GL_STATIC_DRAW_ARB );
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 0        , arrSize, vao->V ); // copy vertices starting from 0 offest
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, arrSize  , arrSize, vao->N ); // copy normals after vertices
    glBufferSubDataARB( GL_ARRAY_BUFFER_ARB, 2*arrSize, arrSize, vao->C ); // copy colours after normals
}

void draw_VAO_VNC_f( VAO_VNC_f* vao ){
    // Draw using "VAO Method" (See Song Ho Ahn code)
    // printf( "About to draw VAO ...\n" );
    uint arrSize = sizeof( float ) * 9 * (vao->Nvtx);
    uint dblSize = arrSize*2;
    // // bind VBOs with IDs and set the buffer offsets of the bound VBOs
    // // When buffer object is bound with its ID, all pointers in gl*Pointer()
    // // are treated as offset instead of real pointer.
    // glBindBufferARB( GL_ARRAY_BUFFER_ARB, vao->bufID );

    // // enable vertex arrays
    // glEnableClientState( GL_NORMAL_ARRAY );
    // glEnableClientState( GL_COLOR_ARRAY  );
    // glEnableClientState( GL_VERTEX_ARRAY );

    // // before draw, specify vertex and index arrays with their offsets
    // glVertexPointer(3, GL_FLOAT, 0, 0);
    // glNormalPointer(GL_FLOAT, 0, &arrSize);
    // glColorPointer(3, GL_FLOAT, 0, &dblSize);

    // glDrawArrays(GL_TRIANGLES, 0, 36);

    // glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    // glDisableClientState(GL_COLOR_ARRAY);
    // glDisableClientState(GL_NORMAL_ARRAY);

    // // it is good idea to release VBOs with ID 0 after use.
    // // Once bound with 0, all pointers in gl*Pointer() behave as real
    // // pointer, so, normal vertex array operations are re-activated
    // glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

    // enable vertex arrays
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);

    // before draw, specify vertex arrays
    glVertexPointer(3, GL_FLOAT, 0, vao->V);
    glNormalPointer(GL_FLOAT, 0, vao->N);
    glColorPointer(3, GL_FLOAT, 0, vao->C);
    

    glDrawArrays(GL_TRIANGLES, 0, vao->Nvtx*3);

    glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}


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


VAO_VNC_f* cube_VAO_VNC_f( void ){
    // Make a colorful cube from the static array data
    printf( "About to allocate cube ...\n" );
    VAO_VNC_f* rtnVAO = make_VAO_VNC_f( 12 );
    printf( "About to populate cube ...\n" );
    load_VAO_VNC_from_full_arrays( rtnVAO, cubeV, cubeN, cubeC );
    return rtnVAO;
}



////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////
VAO_VNC_f* cube = NULL;
Camera3D   cam  = { {4.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };



////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void tick(){
    // Background work
    


    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void display(){
    // Refresh display

    // vec4f center = {0.0f,0.0f,0.0f,1.0f};
    // vec4f sphClr = {0.0, 14.0f/255.0f, (214.0f-75.0f)/255.0f,1.0f};

    //  Erase the window and the depth buffer
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glLoadIdentity();

    look( cam );
    draw_VAO_VNC_f( cube );
    
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
    cube = cube_VAO_VNC_f();
    allocate_and_load_VAO_VNC_at_GPU( cube );
    

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
    delete_VAO_VNC_f( cube );

    printf( "\n### DONE ###\n\n" );
    
    //  Return code
    return 0;
}
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

////////// BEHAVIOR TREES //////////////////////////////////////////////////////////////////////////

///// BT Enums ////////////////////////////////////////////////////////////

enum BT_Status{
  INVALID, // BT should start here
  RUNNING, // Started but not done
  SUCCESS, // Completed without failure
  FAILURE, // Criteria failed
}; 
typedef enum BT_Status Status;

enum BT_Type{
    LEAF, // --- Ignore children
    SEQUENCE, // Run sequentially to first failure
    SELECTOR, // Run sequentially to first success
}; 
typedef enum BT_Type BT_Type;


///// BT Struct ///////////////////////////////////////////////////////////
// YAGNI: PLEASE KEEP THIS AS LIGHT AS POSSIBLE

typedef struct{
    // Variable data passed between behaviors
    Status  status;
    void*   data;
}BT_Pckt;

typedef struct{
    // Cheapest possible BT struct in C
    BT_Type type; // --------------------- How should this node run its children?
    Status  status; // ------------------- Current BT status
    BT_Pckt (*update)( BT_Pckt input ); // Update function
    void*   parent; // ------------------- Container
    uint    Nchld; // -------------------- Number of children directly below this node
    void**  children; // ----------------- Children
    uint    index; // -------------------- Currently-running child
}Behavior;

// Dummy update that always succeeds
BT_Pckt always_succeed( BT_Pckt input ){  BT_Pckt res = {SUCCESS,NULL};  return res;  }

///// BT Methods //////////////////////////////////////////////////////////

Behavior* get_BT_child_i( Behavior* parent, uint i ){
    // Get the `i`th child of this `Behavior`
    if( i < parent->Nchld ){  return (Behavior*) parent->children[i];  }else{  return NULL;  }
}

BT_Pckt tick_once( Behavior* parent, BT_Pckt rootPacket ){
    // Advance the BT by one timestep
    bool    running;
    BT_Pckt res_i;
    /// 0. Init ///
    if( parent->status == INVALID ){
        parent->index = 0;
        parent->status = RUNNING;
    }
    /// 1. Run own update ///
    BT_Pckt result = parent->update( rootPacket );
    /// 2. Run children updates ///
    if( parent->status == RUNNING ){
        switch( parent->type ){
            case LEAF:
                return result;
            case SEQUENCE:
                running = true;
                while( running ){
                    res_i = get_BT_child_i( parent, parent->index )->update( rootPacket );
                    switch( res_i.status ){
                        // FIXME, START HERE: HANDLE CHILD UPDATE RESULT
                        case /* constant-expression */:
                            /* code */
                            break;
                        
                        default:
                            break;
                    }
                    (parent->index)++;
                }
                break;
            default:
                break;
        }
    }else{  parent->status = FAILURE;  }
    /// N. Return ///
    return result;
}

////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////





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
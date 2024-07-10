// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"
#include <time.h>



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
    SEQUENCE, // Run sequentially to first failure (Has memory!)
    SELECTOR, // Run sequentially to first success
}; 
typedef enum BT_Type BT_Type;


///// BT Structs //////////////////////////////////////////////////////////
// YAGNI: PLEASE KEEP THIS AS LIGHT AS POSSIBLE

typedef struct{
    // Variable data passed between behaviors
    Status  status;
    ulong   tickNum;
    void*   data;
}BT_Pckt;


// Default execution packets
BT_Pckt invalid_packet( BT_Pckt input ){  BT_Pckt res = {INVALID, input.tickNum, NULL};  return res;  }
BT_Pckt running_packet( BT_Pckt input ){  BT_Pckt res = {RUNNING, input.tickNum, NULL};  return res;  }
BT_Pckt success_packet( BT_Pckt input ){  BT_Pckt res = {SUCCESS, input.tickNum, NULL};  return res;  }


typedef struct{
    // Cheapest possible BT struct in C
    BT_Type type; // --------------------- How should this node run its children?
    char*   name; // --------------------- Display name of this `Behavior`
    Status  status; // ------------------- Current BT status
    BT_Pckt (*init  )( BT_Pckt input ); // Init   function
    BT_Pckt (*update)( BT_Pckt input ); // Update function
    void*   parent; // ------------------- Container
    uint    Nchld; // -------------------- Number of children directly below this node
    void**  children; // ----------------- Children
    uint    index; // -------------------- Currently-running child
}Behavior;


Behavior* make_action_leaf( char* name_, BT_Pckt (*initFunc  )( BT_Pckt ), BT_Pckt (*updateFunc  )( BT_Pckt ) ){
    // Create a Behavior that performs an action
    Behavior* rtnBhv = malloc( sizeof( Behavior ) );
    rtnBhv->type     = LEAF; // ----- How should this node run its children?
    rtnBhv->name     = name_; // ---- Display name of this `Behavior`
    rtnBhv->status   = INVALID; // -- Current BT status
    rtnBhv->init     = initFunc; // - Init   function
    rtnBhv->update   = updateFunc; // Update function
    rtnBhv->parent   = NULL; // ----- Container
    rtnBhv->Nchld    = 0; // -------- Number of children directly below this node
    rtnBhv->children = NULL; // ----- Children
    rtnBhv->index    = 0; // -------- Currently-running child
    return rtnBhv;
}

Behavior* make_sequence_container( char* name_, uint N_chldrn, 
                                   BT_Pckt (*initFunc  )( BT_Pckt ), BT_Pckt (*updateFunc  )( BT_Pckt ) ){
    // Create a Behavior that executes a sequence of leaf actions
    Behavior* rtnBhv = malloc( sizeof( Behavior ) );
    rtnBhv->type     = SEQUENCE; // ----- How should this node run its children?
    rtnBhv->name     = name_; // ---- Display name of this `Behavior`
    rtnBhv->status   = INVALID; // -- Current BT status
    rtnBhv->init     = initFunc; // - Init   function
    rtnBhv->update   = updateFunc; // Update function
    rtnBhv->parent   = NULL; // ----- Container
    rtnBhv->Nchld    = 0; // -------- Number of children directly below this node
    rtnBhv->children = malloc( N_chldrn * sizeof( Behavior ) ); // ----- Children
    rtnBhv->index    = 0; // -------- Currently-running child
    return rtnBhv;
}

Behavior* make_selector_container( char* name_, uint N_chldrn, 
                                   BT_Pckt (*initFunc  )( BT_Pckt ), BT_Pckt (*updateFunc  )( BT_Pckt ) ){
    // Create a Behavior that executes a sequence of leaf actions
    Behavior* rtnBhv = malloc( sizeof( Behavior ) );
    rtnBhv->type     = SELECTOR; // ----- How should this node run its children?
    rtnBhv->name     = name_; // ---- Display name of this `Behavior`
    rtnBhv->status   = INVALID; // -- Current BT status
    rtnBhv->init     = initFunc; // - Init   function
    rtnBhv->update   = updateFunc; // Update function
    rtnBhv->parent   = NULL; // ----- Container
    rtnBhv->Nchld    = 0; // -------- Number of children directly below this node
    rtnBhv->children = malloc( N_chldrn * sizeof( Behavior ) ); // ----- Children
    rtnBhv->index    = 0; // -------- Currently-running child
    return rtnBhv;
}

uint add_child( Behavior* parent, Behavior* child ){
    // Add `child` under `parent`
    parent->children[ parent->Nchld ] = (void*) child;
    ++(parent->Nchld);
}


// Defualt init that Discards input && Always runs
BT_Pckt default_init( BT_Pckt input ){  return running_packet( input );  }

// Dummy update that Discards input && Always succeeds
BT_Pckt always_succeed( BT_Pckt input ){  return success_packet( input );  }


///// BT Methods //////////////////////////////////////////////////////////

Behavior* get_BT_child_i( Behavior* parent, uint i ){
    // Get the `i`th child of this `Behavior`
    if( i < parent->Nchld ){  return (Behavior*) parent->children[i];  }else{  return NULL;  }
}


BT_Pckt run_init( Behavior* behav, BT_Pckt packet ){
    // Run init && Set status && Return the init result
    BT_Pckt res;
    behav->index  = 0;
    res = behav->init( packet );
    behav->status = res.status;
    return res;
}


BT_Pckt run_update( Behavior* behav, BT_Pckt packet ){
    // Run update && Set status && Return the update result
    BT_Pckt res;
    res = behav->update( packet );
    behav->status = res.status;
    return res;
}


BT_Pckt tick_once( Behavior* behav, BT_Pckt rootPacket ){
    // Advance the BT by one timestep
    Behavior* child_i = NULL; // ----------- Current container child
    BT_Pckt   res_i   = invalid_packet( rootPacket ); // Running execution result
    
    /// 0. Init ///
    if( behav->status == INVALID ){  res_i = run_init( behav, rootPacket );  }
    /// 1. Run updates ///
    if( behav->status == RUNNING ){
        /// 2. Run own update ///
        res_i = run_update( behav, rootPacket );
        /// 3. Handle container types, Run child updates ///
        switch( behav->type ){
            case LEAF: // Update was already run for leaf, return result
                return res_i;
            case SEQUENCE:
                child_i = get_BT_child_i( behav, behav->index );
                if( child_i->status == INVALID ){  res_i = run_init( child_i, rootPacket );  }
                if( child_i->status == RUNNING ){
                    res_i = tick_once( child_i, res_i );
                    switch( res_i.status ){
                        case SUCCESS:
                            (behav->index)++;
                            if( behav->index >= behav->Nchld ){  behav->status = SUCCESS;  }
                            break;
                        case FAILURE:
                            behav->status = FAILURE;
                            break;
                        default:
                            behav->status = res_i.status;
                            break;
                    }
                }
                break;
            case SELECTOR:
                child_i = get_BT_child_i( behav, behav->index );
                if( child_i->status == INVALID ){  res_i = run_init( child_i, rootPacket );  }
                res_i = tick_once( child_i, res_i );
                switch( res_i.status ){
                    case SUCCESS:
                        behav->status = SUCCESS;
                        break;
                    case FAILURE:
                        (behav->index)++;
                        if( behav->index >= behav->Nchld ){  
                            behav->status = FAILURE;  
                        }else{  
                            behav->status = RUNNING;  
                        }
                        break;
                    default:
                        behav->status = res_i.status;
                        break;
                }
                break;
            default:
                printf( "UNHANDLED BEHAVIOR TYPE!: %i", behav->type );
                break;
        }
    }
    /// N. Return ///
    return res_i;
}


// FIXME, START HERE: FUNCTION TO RESET ENTIRE TREE TO `INVALID`


////////// TIME HELPERS ////////////////////////////////////////////////////////////////////////////

inline long get_epoch_nano( void ){
    // Get nanoseconds since the epoch
    // Author: Ciro Santilli, https://stackoverflow.com/a/36095407
    struct timespec ts;
    timespec_get( &ts, TIME_UTC );
    return (long) ts.tv_sec * 1000000000L + ts.tv_nsec;
}

inline double get_epoch_milli( void ){
    // Get milliseconds since the epoch
    return get_epoch_nano() / ((double) 1e6);
}

inline void sleep_ms( double pause_ms ){
    // Main thread will sleep for `pause_ms`
    // Author: Ciro Santilli, https://stackoverflow.com/q/7684359
    struct timespec tim, tim2;
    tim.tv_sec  = (uint) pause_ms / 1000.0;
    tim.tv_nsec = (uint) (pause_ms - tim.tv_sec) * 1000000;
    if( nanosleep( &tim, &tim2 ) < 0 ){  printf( "`nanosleep` system call failed \n" );  }
}



////////// BT RUNNER ///////////////////////////////////////////////////////////////////////////////

typedef struct{
    // Cheapest possible BT manager in C
    Status    status; // Current root status
    ulong     ts; // --- Current timestep/tick
    double    period_ms; // Minimum milliseconds between ticks
    double    lastTick; //- Epoch time of last tick, [ms]
    Behavior* root; // - BT to run
}BT_Runner;


BT_Runner* setup_BT_w_freq( Behavior* root_, double tickHz ){
    // Get a populated `BT_Runner` struct
    BT_Runner* rtnRnnr = malloc( sizeof( BT_Runner ) );
    rtnRnnr->status    = INVALID;
    rtnRnnr->ts /*--*/ = 0;
    rtnRnnr->period_ms = (1.0/tickHz)*1000.0;
    rtnRnnr->lastTick  = get_epoch_milli();
    rtnRnnr->root /**/ = root_;
    return rtnRnnr;
}

void reset_BT( BT_Runner* runner ){
    // Reset the runner
    runner->status   = INVALID;
    runner->ts /*-*/ = 0;
    runner->lastTick = get_epoch_milli();
}

BT_Pckt run_BT_tick( BT_Runner* runner ){
    // Check for elapsed time and run the BT for one tick
    // WARNING: THIS FUNCTION SLEEPS THE THREAD!
    BT_Pckt rtnPkt = {runner->status, runner->ts, NULL};
    double  now    = get_epoch_milli();
    double  elp    = now - (runner->lastTick);

    if( runner->status == INVALID ){
        runner->status = RUNNING;
        rtnPkt.status  = RUNNING;
        printf( "Behavior Tree init!\n" );
    }

    if( runner->status == RUNNING ){
        if( elp < (runner->period_ms) ){  sleep_ms( (runner->period_ms)-elp );  }
        rtnPkt = tick_once( runner->root, rtnPkt );
        ++(runner->ts);
        runner->status   = rtnPkt.status;
        runner->lastTick = get_epoch_milli();
    }

    return rtnPkt;
}


////////// BEHAVIORS ///////////////////////////////////////////////////////////////////////////////


BT_Pckt Countdown_10( BT_Pckt input ){
    // Return `SUCCESS` if 10 ticks have passed, otherwise returning `RUNNING`
    BT_Pckt rtnPkt;
    rtnPkt.data    = NULL;
    rtnPkt.tickNum = input.tickNum;
    if( input.tickNum < 10 ){  rtnPkt.status = RUNNING;  }else{  rtnPkt.status = SUCCESS;  }
    return rtnPkt;
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
#pragma GCC diagnostic ignored "-Wmissing-braces" 

#ifndef TOOLBOX_H // This pattern is to prevent symbols to be loaded multiple times
#define TOOLBOX_H // from multiple imports

////////// INCLUDES & DEFINES //////////////////////////////////////////////////////////////////////

///// Defines /////////////////////////////////////////////////////////////
// NOTE: It's just a good idea to put `#define`s before `#include`s because they might trigger important macros
#define _USE_MATH_DEFINES 
#define GL_GLEXT_PROTOTYPES // REQUIRED HERE: Get all GL prototypes // WARNING: MUST appear BEFORE ALL GL includes!
#define LEN 8192 // ---------- Maximum length of text string


///// Includes ////////////////////////////////////////////////////////////

/// Standard ////
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>  
#include <stdbool.h> // bool

/// OpenGL + GLUT ////
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>



////////// TYPE DEFINES ////////////////////////////////////////////////////////////////////////////

typedef unsigned char ubyte;
typedef unsigned int  uint;
typedef unsigned long ulong;


////////// VECTOR STRUCTS //////////////////////////////////////////////////////////////////////////

typedef struct{
    union{ float x; float r; };
    union{ float y; float g; };
    union{ float z; float b; };
    union{ float w; float a; };
} vec4f;

typedef struct{
  union{ float x; float r; };
  union{ float y; float t; };
} vec2f;

typedef struct{
    union{ uint v0; uint f0; };
    union{ uint v1; uint f1; };
    union{ uint v2; uint f2; };
} vec3u;


////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

typedef struct {
    // Camera state goes here
    vec4f eyeLoc; // Camera location (world frame)
    vec4f lookPt; // Focus of camera (world frame)
    vec4f upVctr; // Direction of "up"
} Camera3D;



////////// BEHAVIOR STRUCTS ////////////////////////////////////////////////////////////////////////
#define _BT_REF_SLOTS    16 // 2024-07-13: At this time, space for state is static
#define _BT_STATE_SLOTS 128 // 2024-07-13: At this time, space for state is static

///// BT Enums ////////////////////////////////////////////////////////////

enum BT_Status{
    // Behavior Status
    INVALID, // BT should start here
    RUNNING, // Started but not done
    SUCCESS, // Completed without failure
    FAILURE, // Criteria failed
}; 
typedef enum BT_Status Status;

enum BT_Type{
    // Behavior Node Type
    LEAF, // --- Ignore children
    SEQUENCE, // Run sequentially to first failure (Has memory!)
    SELECTOR, // Run sequentially to first success
}; 
typedef enum BT_Type BT_Type;


///// BT Structs //////////////////////////////////////////////////////////

typedef struct{
    // Variable data passed between behaviors
    Status  status;
    ulong   tickNum;
}BT_Pckt;


typedef struct{
    // Cheapest possible BT struct in C
    BT_Type type; // ---------------------- How should this node run its children?
    char*   name; // ---------------------- Display name of this `Behavior`
    Status  status; // -------------------- Current BT status
    void**  refs; // ---------------------- Structures needed by this `Behavior`
    float*  state; // --------------------- Data specific to this `Behavior`
    BT_Pckt (*init  )( void*, BT_Pckt ); // Init   function
    BT_Pckt (*update)( void*, BT_Pckt ); // Update function
    void*   parent; // -------------------- Container
    uint    Nchld; // --------------------- Number of children directly below this node
    void**  children; // ------------------ Children
    uint    index; // --------------------- Currently-running child
}Behavior;


///// BT Runner ///////////////////////////////////////////////////////////

typedef struct{
    // Cheapest possible BT manager in C
    Status    status; // -- Current root status
    uint      done; // ---- Has the behavior completed?
    ulong     ts; // ------ Current timestep/tick
    double    period_ms; // Minimum milliseconds between ticks
    double    lastTick; //- Epoch time of last tick, [ms]
    Behavior* root; // ---- BT to run
}BT_Runner;



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// OGL_utils.c ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x );
double Sin( double x );
double Tan( double x );
float  Cosf( float x );
float  Sinf( float x );
float  Tanf( float x );
float  Atan2f( float y, float x );


////////// MATH HELPERS ////////////////////////////////////////////////////////////////////////////
uint min_uint( uint x, uint y );
uint max_uint( uint x, uint y );

///// Random Numbers //////////////////////////////////////////////////////
void init_rand(); // Init random seed from clock

float randf(); // ------------------------ Return a pseudo-random float between 0.0 and 1.0
float randf_range( float lo, float hi );// Return a pseudo-random float between `lo` and `hi`
int   randi_range( int lo, int hi ); // -- Return a pseudo-random int   between `lo` and `hi` 
uint  randu_range( uint lo, uint hi ); //- Return a pseudo-random uint  between `lo` and `hi` 
ubyte rand_ubyte(); // ------------------- Return a pseudo-random ubyte



////////// SIMULATION & TIMING /////////////////////////////////////////////////////////////////////
// Pause main thread: implemented using nanosleep(), continuing the sleep if it is interrupted by a signal
int sleep_ms( long msec ); 
// Attempt to maintain framerate no greater than target. (Period is rounded down to next ms)
float heartbeat_FPS( float targetFPS );
double get_epoch_milli( void ); // Get milliseconds since the epoch

////////// CAMERA //////////////////////////////////////////////////////////////////////////////////
void look( const Camera3D camera ); // Set camera position, target, and orientation
// Move the `camera` in a circular arc by `delTheta_deg` in the X-Y plane w/ `camera.lookPt` as the center
void orbit_target_about_Z( Camera3D* camera, float delTheta_deg );
    


////////// OPENGL SYSTEM ///////////////////////////////////////////////////////////////////////////
// Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/

void ErrCheck( const char* where ); // --------------- Check for OpenGL errors and print to stderr
void Fatal( const char* format, ... ); // ------------ Print message to stderr and exit
void Print( const char* format, ... ); // ------------ Print raster letters to the viewport
void Project( double fov, double asp, double dim ); // Set projection



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// load_assets.c //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/

unsigned int LoadTexBMP( const char* file );
int  LoadOBJ( const char* file );


////////// PRINTING HELPERS ////////////////////////////////////////////////////////////////////////
void nl( void ); // Emit a newline to console



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// vector-f_ops.c /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// SCALED 3D VECTORS ///////////////////////////////////////////////////////////////////////

vec4f make_vec4f( float x, float y, float z ); // Make a 3D float vector with scale = 1.0 from three floats
vec4f make_0_vec4f( void ); // ------------------ Make a 3D zero float vector with scale = 1.0
vec4f make_0w0_vec4f( void ); // ---------------- Make a 3D zero float vector with scale = 0.0
vec4f rand_vec4f( void ); // -------------------- Make a random 3D float vector with each of {X,Y,Z} on (0.0, 1.0]

vec4f sub_vec4f( const vec4f u, const vec4f v ); // --------------------- Calc `u` - `v` = `r`, R^3
vec4f add_vec4f( const vec4f u, const vec4f v ); // --------------------- Calc `u` + `v` = `r`, R^3
float dot_vec4f( const vec4f u, const vec4f v ); // --------------------- Calc `u` * `v` = `r`, R^3
float norm_vec4f( const vec4f vec ); // --------------------------------- Euclidean length of an R^3
float diff_vec4f( const vec4f u, const vec4f v ); // -------------------- Euclidean length of `u`-`v`
vec4f unit_vec4f( const vec4f vec ); // --------------------------------- Calc the unit direction of `vec` and return it, R^3
vec4f cross_vec4f( const vec4f u, const vec4f v ); // ------------------- Calc `u` X `v` = `p`, R^3
vec4f div_vec4f( const vec4f u, float d ); // --------------------------- Calc `u` / `f` = `r`, R^3
vec4f scale_vec4f( const vec4f u, float f ); // ------------------------- Calc `u` * `f` = `r`, R^3
vec4f blend_vec4f( const vec4f u, float fU, const vec4f v, float fV ); // Return the weighted sum of the two verctors, R^3
vec4f stretch_to_len_vec4f( const vec4f vec, float len ); // ------------ Stretch `vec` to `len` and return
float angle_between_vec4f( const vec4f vec1, const vec4f vec2 ); // Get the angle between two R3 vectors, radians
    

///// 3D Segments & Triangles /////////////////////////////////////////////

vec4f seg_center( const vec4f v0, const vec4f v1 ); // --------------- Calc centroid of 2 R^3 points
vec4f tri_center( const vec4f v0, const vec4f v1, const vec4f v2 ); // Calc centroid of 3 R^3 points
// Find the normal vector `n` of a triangle defined by CCW vertices in R^3: {`v0`,`v1`,`v2`}
vec4f get_CCW_tri_norm( const vec4f v0, const vec4f v1, const vec4f v2 ); 


////////// 2D VECTORS //////////////////////////////////////////////////////////////////////////////

vec2f make_vec2f( float x, float y ); // Create a 2D float vector from 2 floats


////////// 2D <---> 3D /////////////////////////////////////////////////////////////////////////////

// Project the local 2D point to the global 3D frame
vec4f lift_pnt_2D_to_3D( const vec2f pnt2f, const vec4f origin, const vec4f xBasis, const vec4f yBasis ); 
// Project the local 2D vector to the global 3D frame
vec4f lift_vec_2D_to_3D( const vec2f vct2f, const vec4f xBasis, const vec4f yBasis );


////////// UINT VECTORS ////////////////////////////////////////////////////////////////////////////

vec3u make_vec3u( uint u0, uint u1, uint u2 ); // Create a 3D uint vector

vec2f add_vec2f( const vec2f u, const vec2f v ); // Calc `u` + `v` = `r`, R^2


////////// VECTOR PRINTING /////////////////////////////////////////////////////////////////////////

void print_vec4f( const vec4f vec );
void print_vec2f( const vec2f vec );
void print_vec3u( const vec3u vec );


////////// OPENGL HELPERS //////////////////////////////////////////////////////////////////////////

void glVtx4f( const vec4f v ); // Set vertex with a vector
void glNrm4f( const vec4f n ); // Set normal with a vector
void glClr4f( const vec4f c ); // Set color with a vector



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// shaders.c //////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/

////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////

char* ReadText( const char *file ); // Read the contents of a text file


////////// SHADER STATUS ///////////////////////////////////////////////////////////////////////////

void PrintShaderLog( int obj, const char* file ); // Attempt to retrieve compilation status of the shader program
void PrintProgramLog( int obj ); // ---------- Print Program Log


////////// SHADER INSTANTIATION ////////////////////////////////////////////////////////////////////

int CreateShader(int prog, const GLenum type, const char* file); // Create the shader program and return its ID
// Create the shader program, check it for errors, and return the ID
int CreateShaderProg( const char* VertFile, const char* FragFile ); 
// Create a complete Vertex --> Geometry --> Fragment shader pipeline
int CreateShaderGeom( const char* VertFile, const char* GeomFile, const char* FragFile );
int CreateShaderProgCompute( const char* file ); // Create a comput shader
    


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// behavior.c /////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// BEHAVIOR TREES //////////////////////////////////////////////////////////////////////////

// Default execution packets
BT_Pckt invalid_packet( void* behav, BT_Pckt input );
BT_Pckt running_packet( void* behav, BT_Pckt input );
BT_Pckt success_packet( void* behav, BT_Pckt input );
BT_Pckt failure_packet( void* behav, BT_Pckt input );

// Create a Behavior that performs an action
Behavior* make_action_leaf( char* name_, BT_Pckt (*initFunc)( void*, BT_Pckt ), BT_Pckt (*updateFunc)( void*, BT_Pckt ) );
// Create a Behavior that executes a sequence of leaf actions
Behavior* make_sequence_container( char* name_, uint N_chldrn ); 
// Create a Behavior that executes a sequence of leaf actions
Behavior* make_selector_container( char* name_, uint N_chldrn );

void add_child( Behavior* parent, Behavior* child ); // Add `child` under `parent`

BT_Pckt pass_packet( void* behav, BT_Pckt input ); // -- Return the input packet without modification
BT_Pckt default_init( void* behav, BT_Pckt input ); // - Defualt init that Discards input && Always runs
BT_Pckt always_succeed( void* behav, BT_Pckt input ); // Dummy update that Discards input && Always succeeds


///// BT Methods //////////////////////////////////////////////////////////
Behavior* get_BT_child_i( Behavior* parent, uint i ); // ----- Get the `i`th child of this `Behavior`
BT_Pckt   tick_once( Behavior* behav, BT_Pckt rootPacket ); // Advance the BT by one timestep
// Set behavior and all children to `INVALID` status, Set containers to initial child
void reset_tree( Behavior* root ); 


////////// BT RUNNER ///////////////////////////////////////////////////////////////////////////////

BT_Runner* setup_BT_w_freq( Behavior* root_, double tickHz ); // Get a populated `BT_Runner` struct
void /*-*/ reset_BT( BT_Runner* runner ); // ------------------- Reset the runner and associated BT
BT_Pckt    run_BT_tick( BT_Runner* runner ); // ---------------- Check time elapsed and conditionally run the BT for one tick
    

#endif

// gcc -std=gnu17 -O3 -Wall 03_atmos-CPU.c -lglut -lGLU -lGL -lm -o atmos.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "TriNet.h"
#include "OGL_Geo.h"
#include "OGL_Tools.h"


////////// VIEW SETTINGS & WINDOW STATE ////////////////////////////////////////////////////////////
const float _SCALE   = 10.0; //- Scale Dimension
float /*-*/ w2h /**/ =  0.0f; // Aspect ratio
int   /*-*/ fov /**/ = 55; // -- Field of view (for perspective)
float /*-*/ lastTime = 0.0f; //- Time of last frame since GLUT start [s]



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// Geometry ///
const float _SPHERE_RADIUS = 2.15f;
const float _ATMOS_RADIUS  = 2.25f;
const uint  _ICOS_SUBDIVID = 6;

/// Init ///
const uint  _N_PARTICLES  = 2000000;
const uint  _N_ATMOS_NATR =      25;
const uint  _N_WARM_UP    =      65;

/// Dynamics ///
const float _SPEED_LIMIT  =   0.0075;
const float _ACCEL_LIMIT  =   0.00020;
const float _DIFFUS_PROB  = 1.0f/2000.0f;
const float _DIFFUS_RATE  = 0.0625;
const float _PERTURB_PROB = 1.0f/25.0f;
const float _PERTURB_RATE = 0.75;



////////// GLOBAL PROGRAM GEOMETRY & STATE /////////////////////////////////////////////////////////
uint    N_cells = 0;

int shader_ID; // Shader program

/// Cell per Row ///
uint origin_ID;
uint v1_ID;
uint v2_ID;
uint xBasis_ID;
uint yBasis_ID;
uint accel2f_ID;
uint nghbrs_ID;

/// Particle per Row ///
uint posnArr_ID; // Position array
uint veloArr_ID; // Velocity array
uint colrArr_ID; // Color    array



////////// GPU HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////

GPtr_vec4 get_vec4_arr_from_buffer_obj( uint N ){
    GPtr_vec4 handle = {0, NULL};
    // Reset position
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, handle.ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    handle.arr = (vec4*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N*sizeof( vec4 ),
                                           GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    return handle;
}

void release_buffer_obj(){  glUnmapBuffer( GL_SHADER_STORAGE_BUFFER );  } // Release buffer object



////////// ATMOSPHERE CONSTRUCTION & GPU MEMORY OPS ////////////////////////////////////////////////

void allocate_cell_memory_at_GPU(){
    // Construct geo and move it to the GPU
    vec4*     target;
    GPtr_vec4 handle;
    TriNet*   icosphere = create_icosphere_VFNA( _ATMOS_RADIUS, _ICOS_SUBDIVID );
    vec4*     orgnArr   = NULL;
    vec4*     v1_Arr    = NULL;

    N_cells = icosphere->Ntri;
    orgnArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    v1_Arr  = (vec4*) malloc( N_cells * sizeof( vec4 ) );

    /// Origin ///
    handle = get_vec4_arr_from_buffer_obj( N_cells );
    origin_ID = handle.ID;
    target    = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        orgnArr[i].x = target[i].x = (*icosphere->V)[ (*icosphere->F)[i][0] ][0];
        orgnArr[i].y = target[i].y = (*icosphere->V)[ (*icosphere->F)[i][0] ][1];
        orgnArr[i].z = target[i].z = (*icosphere->V)[ (*icosphere->F)[i][0] ][2];
        orgnArr[i].w = target[i].w = 1.0f;
    }
    release_buffer_obj();

    /// Vertex 1 ///
    handle = get_vec4_arr_from_buffer_obj( N_cells );
    v1_ID  = handle.ID;
    target = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        v1_Arr[i].x = target[i].x = (*icosphere->V)[ (*icosphere->F)[i][1] ][0];
        v1_Arr[i].y = target[i].y = (*icosphere->V)[ (*icosphere->F)[i][1] ][1];
        v1_Arr[i].z = target[i].z = (*icosphere->V)[ (*icosphere->F)[i][1] ][2];
        v1_Arr[i].w = target[i].w = 1.0f;
    }
    release_buffer_obj();

    /// Vertex 2 ///
    handle = get_vec4_arr_from_buffer_obj( N_cells );
    v2_ID  = handle.ID;
    target = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        target[i].x = (*icosphere->V)[ (*icosphere->F)[i][2] ][0];
        target[i].y = (*icosphere->V)[ (*icosphere->F)[i][2] ][1];
        target[i].z = (*icosphere->V)[ (*icosphere->F)[i][2] ][2];
        target[i].w = 1.0f;
    }
    release_buffer_obj();

    /// X Basis ///
    handle = get_vec4_arr_from_buffer_obj( N_cells );
    v2_ID  = handle.ID;
    target = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        target[i] = unit_vec4( sub_vec4( v1_Arr[i], orgnArr[i] ) );
    }

    /// Y Basis ///
    // FIXME, START HERE: CALC Y BASIS

    // N. Cleanup
    delete_net( icosphere );
    free( orgnArr );
    free( v1_Arr );
}

// gcc -std=gnu17 -O3 -Wall 05_atmos-GPU.c -lglut -lGLU -lGL -lm -o atmosGPU.out
#pragma GCC diagnostic ignored "-Wmissing-braces"

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
const float _ACCEL_MIN    =   0.00005;
const float _DIFFUS_PROB  = 1.0f/2000.0f;
const float _DIFFUS_RATE  = 0.0625;
const float _PERTURB_PROB = 1.0f/25.0f;
const float _PERTURB_RATE = 0.75;



////////// GLOBAL PROGRAM GEOMETRY & STATE /////////////////////////////////////////////////////////
uint N_cells = 0;
int  shader_ID; // Shader program

/// Cell per Row ///
uint origin_ID;
uint v1_ID;
uint v2_ID;
uint xBasis_ID;
uint yBasis_ID;
uint accel_ID;
uint nghbrs_ID;

/// Particle per Row ///
uint posnArr_ID; // Position array
uint veloArr_ID; // Velocity array
uint colrArr_ID; // Color    array



////////// GPU HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////

GPtr_vec4 get_vec4_arr_from_buffer_obj( unsigned long int N ){
    // Allocate space on the GPU for an array of `vec4` of length `N`
    GPtr_vec4 handle = {0, NULL};
    // Reset position
    int res = glBindBuffer( GL_SHADER_STORAGE_BUFFER, handle.ID ); // Ask buffer object for a new buffer ID
    // Get pointer to buffer and cast as a struct array
    handle.arr = (vec4*) (long) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N*sizeof( vec4 ),
                                                  GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    printf( "Allocated %lu bytes on the GPU for `vec4`, Result: %i" , N*sizeof( vec4 ), res );
    return handle;
}

GPtr_vec3uu get_vec3uu_arr_from_buffer_obj( unsigned long int N ){
    // Allocate space on the GPU for an array of `vec4` of length `N`
    GPtr_vec3uu handle = {0, NULL};
    // Reset position
    int res = glBindBuffer( GL_SHADER_STORAGE_BUFFER, handle.ID ); // Ask buffer object for a new buffer ID
    // Get pointer to buffer and cast as a struct array
    handle.arr = (vec3uu*) (long) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N*sizeof( vec3uu ),
                                                    GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    printf( "Allocated %lu bytes on the GPU for `vec3uu`, Result: %i" , N*sizeof( vec3uu ), res );
    return handle;
}

void release_buffer_obj(){  glUnmapBuffer( GL_SHADER_STORAGE_BUFFER );  } // Release buffer object

void halt_buffer_obj(){  glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );  } // Stop talking to the buffer object?



////////// INIT HELPERS ////////////////////////////////////////////////////////////////////////////

float Box_Muller_normal_sample( float mu, float sigma ){
    // Transform 2 uniform samples into a zero-mean normal sample
    // Source: https://www.baeldung.com/cs/uniform-to-normal-distribution
    float u1 = randf();
    float u2 = randf();
    return mu + sqrtf( -2.0 * log( u1 ) ) * cos( 2.0 * M_PI * u2 ) * sigma;
}


uint* distribute_particles_init( vec4* prtArr, vec4* orgnArr, vec4* xBasArr, vec4* yBasArr, float scale,
                                uint groupSizeMin, uint groupSizeMax ){
    // Place particles in the atmosphere
    // 0. Init
    uint* rtnArr  = (uint*) malloc( _N_PARTICLES * sizeof( uint ) );
    uint  cellDex = 0;
    uint  grpSize = 0;
    uint  j /*-*/ = 0;
    bool  onGroup = false;
    vec4  v0 /**/ = {0.0f,0.0f,0.0f,1.0f};
    vec4  v1 /**/ = {0.0f,0.0f,0.0f,1.0f};
    vec2  posn2f  = {0.0f,0.0f};
    vec2  center  = {0.0f,0.0f};
    // 1. For each particle in the simulation
    for( uint i = 0; i < _N_PARTICLES; ++i ){
        // 2. If there is no active cluster, then start one
        if( !onGroup ){
            cellDex  = randu_range( 0, N_cells-1 );
            grpSize  = randu_range( groupSizeMin, groupSizeMax );
            j /*--*/ = 0;
            center.x = randf() * scale;
            center.y = randf() * scale;
            onGroup  = true;
        }
        // 3. Distribute point about cluster center w/ 2D Gaussian
        posn2f.x = Box_Muller_normal_sample( 0.0f, scale/8.0f );
        posn2f.y = Box_Muller_normal_sample( 0.0f, scale/8.0f );
        posn2f   = add_vec2( center, posn2f );
        // 4. Place point in 3D
        prtArr[i] = lift_pnt_2D_to_3D( posn2f, orgnArr[ cellDex ], xBasArr[ cellDex ], yBasArr[ cellDex ] );
        rtnArr[i] = cellDex;
        // 5. Increment group counter and check if group is done
        ++j;
        if( j >= grpSize ){  onGroup = false;  }
    }
}



////////// ATMOSPHERE CONSTRUCTION & GPU MEMORY OPS ////////////////////////////////////////////////

void allocate_cell_memory_at_GPU(){
    // Construct geo and move it to the GPU
    vec4* /*-*/ target;
    vec3uu*     targetU;
    GPtr_vec4   handle;
    GPtr_vec3uu handleU;
    TriNet*     icosphr   = create_icosphere_VFNA( _ATMOS_RADIUS, _ICOS_SUBDIVID );
    vec4* /*-*/ orgnArr   = NULL;
    vec4* /*-*/ v1_Arr    = NULL;
    vec4* /*-*/ xBasArr   = NULL;
    vec4* /*-*/ yBasArr   = NULL;
    uint* /*-*/ memberDex = NULL;
    vec4 /*--*/ norm_i    = {0.0f,0.0f,0.0f,1.0f};
    vec2 /*--*/ accl_i    = {0.0f,0.0f};

    N_cells = icosphr->Ntri;
    orgnArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    v1_Arr  = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    xBasArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    yBasArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );


    ///// Cell per Row ////////////////////////////////////////////////////

    /// Origin ///
    handle    = get_vec4_arr_from_buffer_obj( N_cells );
    origin_ID = handle.ID;
    target    = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        orgnArr[i].x = target[i].x = (*icosphr->V)[ (*icosphr->F)[i][0] ][0];
        orgnArr[i].y = target[i].y = (*icosphr->V)[ (*icosphr->F)[i][0] ][1];
        orgnArr[i].z = target[i].z = (*icosphr->V)[ (*icosphr->F)[i][0] ][2];
        orgnArr[i].w = target[i].w = 1.0f;
    }
    release_buffer_obj();

    /// Vertex 1 ///
    handle = get_vec4_arr_from_buffer_obj( N_cells );
    v1_ID  = handle.ID;
    target = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        v1_Arr[i].x = target[i].x = (*icosphr->V)[ (*icosphr->F)[i][1] ][0];
        v1_Arr[i].y = target[i].y = (*icosphr->V)[ (*icosphr->F)[i][1] ][1];
        v1_Arr[i].z = target[i].z = (*icosphr->V)[ (*icosphr->F)[i][1] ][2];
        v1_Arr[i].w = target[i].w = 1.0f;
    }
    release_buffer_obj();

    /// Vertex 2 ///
    handle = get_vec4_arr_from_buffer_obj( N_cells );
    v2_ID  = handle.ID;
    target = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        target[i].x = (*icosphr->V)[ (*icosphr->F)[i][2] ][0];
        target[i].y = (*icosphr->V)[ (*icosphr->F)[i][2] ][1];
        target[i].z = (*icosphr->V)[ (*icosphr->F)[i][2] ][2];
        target[i].w = 1.0f;
    }
    release_buffer_obj();

    /// X Basis ///
    handle    = get_vec4_arr_from_buffer_obj( N_cells );
    xBasis_ID = handle.ID;
    target    = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        xBasArr[i] = target[i] = unit_vec4( sub_vec4( v1_Arr[i], orgnArr[i] ) );
    }
    release_buffer_obj();

    /// Y Basis ///
    handle    = get_vec4_arr_from_buffer_obj( N_cells );
    yBasis_ID = handle.ID;
    target    = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        norm_i.x = (*icosphr->N)[i][0];
        norm_i.y = (*icosphr->N)[i][1];
        norm_i.z = (*icosphr->N)[i][2];
        yBasArr[i] = target[i] = unit_vec4( cross_vec4( norm_i, xBasArr[i] ) );
    }
    release_buffer_obj();

    /// Acceleration ///
    handle   = get_vec4_arr_from_buffer_obj( N_cells );
    accel_ID = handle.ID;
    target   = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){
        accl_i.x = randf_range( -_ACCEL_LIMIT, +_ACCEL_LIMIT );
        accl_i.y = randf_range( -_ACCEL_LIMIT, +_ACCEL_LIMIT );
        if( norm_vec2( accl_i ) < _ACCEL_MIN ){  accl_i = stretch_vec2_to_len( accl_i, _ACCEL_MIN );  }
        target[i] = lift_vec_2D_to_3D( accl_i, xBasArr[i], yBasArr[i] );
    }
    release_buffer_obj();

    /// Neighbors ///
    handleU   = get_vec3uu_arr_from_buffer_obj( N_cells );
    nghbrs_ID = handleU.ID;
    targetU   = handleU.arr;
    for( uint i = 0; i < N_cells; ++i ){
        targetU[i].f0 = (*icosphr->A)[i][0];
        targetU[i].f1 = (*icosphr->A)[i][1];
        targetU[i].f2 = (*icosphr->A)[i][2];
    }
    release_buffer_obj();


    ///// Particle per Row ////////////////////////////////////////////////

    /// Position ///
    handle     = get_vec4_arr_from_buffer_obj( _N_PARTICLES );
    posnArr_ID = handle.ID;
    target     = handle.arr;
    memberDex  = distribute_particles_init( 
        target, orgnArr, xBasArr, yBasArr, diff_vec4( orgnArr[0], v1_Arr[0] ), 200, _N_PARTICLES/1000 
    );
    release_buffer_obj();

    /// Velocity ///
    handle     = get_vec4_arr_from_buffer_obj( _N_PARTICLES );
    veloArr_ID = handle.ID;
    target     = handle.arr;
    for( uint i = 0; i < N_cells; ++i ){

    }

    halt_buffer_obj();

    // N. Cleanup
    delete_net( icosphr );
    free( orgnArr   );
    free( v1_Arr    );
    free( xBasArr   );
    free( yBasArr   );
    free( memberDex );
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main( int argc , char* argv[] ){
    init_rand();

}
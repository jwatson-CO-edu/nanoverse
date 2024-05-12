////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _SCALE /**/ =  10.0; // Scale Dimension
const int   _FOV_DEG    =  55; // - Field of view (for perspective)
const float _TARGET_FPS =  60.0f; // Desired framerate

/// Geometry ///
const float _SPHERE_RADIUS = 2.15f;
const float _ATMOS_RADIUS  = 2.25f;
const uint  _ICOS_SUBDIVID = 6;
const ulong N_cells /*--*/ = 20 * (_ICOS_SUBDIVID*(_ICOS_SUBDIVID+1)/2 + (_ICOS_SUBDIVID-1)*(_ICOS_SUBDIVID)/2);

/// Init ///
const uint _N_PARTICLES  = 600000;
const uint _N_ATMOS_NATR =      25;
const uint _N_WARM_UP    =      65;

/// Dynamics ///
const float _SPEED_LIMIT  =   0.0075;
const float _ACCEL_LIMIT  =   0.00020;
const float _ACCEL_MIN    =   0.00005;
const float _DIFFUS_PROB  = 1.0f/2000.0f;
const float _DIFFUS_RATE  = 0.0625;
const float _PERTURB_PROB = 1.0f/25.0f;
const float _PERTURB_RATE = 0.75;


////////// GLOBAL PROGRAM STATE & GEOMETRY /////////////////////////////////////////////////////////

///// CPU-Side State //////////////////////////////////////////////////////
Camera3D cam = { {4.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };

vec4f* orgnArr = NULL;
vec4f* xBasArr = NULL;
vec4f* yBasArr = NULL;
vec4f* acclArr = NULL;
vec3u* ngbrArr = NULL;


///// GPU-Side State //////////////////////////////////////////////////////
int workGroupSize;
int N_groups;
int shaderDyna_ID;
int shaderUpdt_ID;

/// Particle per Row ///
uint posnArr_ID; //  Position buffer
uint veloArr_ID; //  Velocity buffer
uint colrArr_ID; //  Color buffer
uint mmbrArr_ID;

/// Cell per Row ///
uint origin_ID;
uint v1_ID;
uint v2_ID;
// uint xBasis_ID;
// uint yBasis_ID;
uint accel_ID;
uint nghbrs_ID;



////////// SCRATCH SPACE ///////////////////////////////////////////////////////////////////////////

// bool p_pnt_vertical_of_face( const vec4f q, const vec4f cntr, const vec4f v0, const vec4f v1, const vec4f v2 ){
//     // For a convex poyhedron with `cntr`, Return true if `q` is above/below/on the triangle {`v0`,`v1`,`v2`}
//     vec4f seg0, seg1, seg2, segCen0, segCen1, segCen2, norm0, norm1, norm2, diff0, diff1, diff2;
//     seg0    = sub_vec4f( v1, v0 );
//     seg1    = sub_vec4f( v2, v1 );
//     seg2    = sub_vec4f( v0, v2 );
//     segCen0 = sub_vec4f( seg_center( v0, v1 ), cntr );
//     segCen1 = sub_vec4f( seg_center( v1, v2 ), cntr );
//     segCen2 = sub_vec4f( seg_center( v2, v0 ), cntr );
//     norm0   = cross_vec4f( segCen0, seg0 );
//     norm1   = cross_vec4f( segCen1, seg1 );
//     norm2   = cross_vec4f( segCen2, seg2 );
//     diff0   = sub_vec4f( sub_vec4f( q, cntr ), segCen0 );
//     diff1   = sub_vec4f( sub_vec4f( q, cntr ), segCen1 );
//     diff2   = sub_vec4f( sub_vec4f( q, cntr ), segCen2 );
//     return (  
//         (dot_vec4f( diff0, norm0 ) >= 0.0f) && (dot_vec4f( diff1, norm1 ) >= 0.0f) && (dot_vec4f( diff2, norm2 ) >= 0.0f) 
//     );
// }


////////// INIT HELPERS ////////////////////////////////////////////////////////////////////////////

float Box_Muller_normal_sample( float mu, float sigma ){
    // Transform 2 uniform samples into a zero-mean normal sample
    // Source: https://www.baeldung.com/cs/uniform-to-normal-distribution
    float u1 = randf();
    float u2 = randf();
    return mu + sqrtf( -2.0 * log( u1 ) ) * cos( 2.0 * M_PI * u2 ) * sigma;
}


uint* distribute_particles_init( vec4f* prtArr, vec4f* orgnArr, vec4f* xBasArr, vec4f* yBasArr, float scale,
                                 uint groupSizeMin, uint groupSizeMax ){
    // Place particles in the atmosphere
    // 0. Init
    uint*  rtnArr  = (uint*) malloc( _N_PARTICLES * sizeof( uint ) );
    uint   cellDex = 0;
    uint   grpSize = 0;
    uint   j /*-*/ = 0;
    bool   onGroup = false;
    vec2f  posn2f  = {0.0f,0.0f};
    vec2f  center  = {0.0f,0.0f};
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
        posn2f   = add_vec2f( center, posn2f );
        // 4. Place point in 3D
        prtArr[i] = lift_pnt_2D_to_3D( posn2f, orgnArr[ cellDex ], xBasArr[ cellDex ], yBasArr[ cellDex ] );
        rtnArr[i] = cellDex;
        // print_vec4f( prtArr[i] );
        // 5. Increment group counter and check if group is done
        ++j;
        if( j >= grpSize ){  
            onGroup = false;  
        }
    }
    return rtnArr;
}

////////// GPU SETUP ///////////////////////////////////////////////////////////////////////////////

void ResetParticles(){
    // Write init data to buffers on the GPU
    // Adapted from code by Willem A. (Vlakkies) Schreüder

    float red_i, grn_i, blu_i;
    vec4f norm_i;
    vec4f *pos, *vel, *col, *trgtVf;
    vec3u* trgtVu;
    uint*  trgtU;
    vec4f* v1_Arr    = NULL;
    vec4f* v2_Arr    = NULL;
    TriNet* icosphr  = create_icosphere_VFNA( _ATMOS_RADIUS, _ICOS_SUBDIVID );
    float   edgScale = avg_edge_len( icosphr );
    uint*   mmbrArr  = NULL;

    printf( "About to allocate CPU array memory ...\n" );
    orgnArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    xBasArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    yBasArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    acclArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    ngbrArr = (vec3u*) malloc( N_cells * sizeof( vec3u ) );
    printf( "About to allocate CPU temp init memory ...\n" );
    v1_Arr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    v2_Arr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );

    

    //  Reset velocities
    printf( "About to set particle velocity, buffer %u ...\n", veloArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, veloArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    vel = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( vec4f ), 
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load init velocities into buffer
    for( int i = 0; i < _N_PARTICLES; i++ ){
        vel[i] = make_0_vec4f();
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    //  Reset colors
    printf( "About to set particle colors, buffer %u ...\n", colrArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, colrArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    col = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( vec4f ), 
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( int i = 0; i < _N_PARTICLES; i++ ){
        grn_i = randf_range( 0.1f, 1.0f );
        blu_i = randf_range( 0.1f, 1.0f );
        red_i = (grn_i + blu_i)/2.0f;
        col[i].r = red_i;
        col[i].g = grn_i;
        col[i].b = blu_i;
        col[i].a = 1.0f;
        // print_vec4f( col[i] );
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell Origin ////////////////////////////

    printf( "About to set cell origins, buffer %u ...\n", origin_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, origin_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        orgnArr[i] = trgtVf[i] = icosphr->V[ icosphr->F[i].v0 ];
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell V1 ////////////////////////////////

    printf( "About to set cell v1, buffer %u ...\n", v1_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, v1_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        v1_Arr[i] = trgtVf[i] = icosphr->V[ icosphr->F[i].v1 ];
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell V2 ////////////////////////////////

    printf( "About to set cell v2, buffer %u ...\n", v2_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, v2_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        v2_Arr[i] = trgtVf[i] = icosphr->V[ icosphr->F[i].v2 ];
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell X Basis ///////////////////////////
    
    // printf( "About to set cell X Basis, buffer %u ...\n", xBasis_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, xBasis_ID ); // Set buffer object to point to this ID, for writing
    // // Get pointer to buffer and cast as a struct array
    // trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
    //                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        xBasArr[i] = unit_vec4f( sub_vec4f( v1_Arr[i], orgnArr[i] ) );
    }
    // glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell Y Basis ///////////////////////////

    // printf( "About to set cell y Basis, buffer %u ...\n", yBasis_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, yBasis_ID ); // Set buffer object to point to this ID, for writing
    // // Get pointer to buffer and cast as a struct array
    // trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
    //                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        norm_i = get_CCW_tri_norm( orgnArr[i], v1_Arr[i], v2_Arr[i] );
        yBasArr[i] = cross_vec4f( norm_i, xBasArr[i] );
    }
    // glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Particle Positions /////////////////////

    printf( "About to set bind buffer %u ...\n", posnArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, posnArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    printf( "About to map buffer %u ...\n", posnArr_ID );
    ErrCheck( "ResetParticles" );
    pos = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( vec4f ),
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    printf( "Sending particles to %p ...\n", pos );
    // Load init positions into buffer
    mmbrArr = distribute_particles_init( pos, orgnArr, xBasArr, yBasArr, edgScale, 
                                         50, 200 );
                                        //  10, 20 );
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Particle Membership ////////////////////

    printf( "About to set bind buffer %u ...\n", mmbrArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, mmbrArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    printf( "About to map buffer %u ...\n", mmbrArr_ID );
    ErrCheck( "ResetParticles" );
    trgtU = (uint*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( uint ),
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    printf( "Sending particles to %p ...\n", pos );
    // Load init positions into buffer
    for( int i = 0; i < _N_PARTICLES; i++ ){
        trgtU[i] = mmbrArr[i];
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell Neighbors ///////////////////////////

    printf( "About to set cell y Basis, buffer %u ...\n", nghbrs_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, nghbrs_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVu = (vec3u*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec3u ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        ngbrArr[i] = trgtVu[i] = icosphr->A[i];
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// Cell Acceleration //////////////////////

    vec2f acl = {0.0f,0.0f};

    printf( "About to set cell acceleration, buffer %u ...\n", accel_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, accel_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){

        acl.x = randf_range( -_ACCEL_LIMIT, +_ACCEL_LIMIT );
        acl.y = randf_range( -_ACCEL_LIMIT, +_ACCEL_LIMIT );

        acclArr[i] = trgtVf[i] = lift_vec_2D_to_3D( acl, xBasArr[i], yBasArr[i] );
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    ///// GPU ALLOC COMPLETE /////////////////////


    ///// Associate buffer ID on GPU side with buffer ID on CPU side //////
    
    ///// Row per Particle ///////////////////////
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  4, posnArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  5, veloArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  6, colrArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  7, mmbrArr_ID );

    ///// Row per Cell ///////////////////////////
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  8, origin_ID  );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  9, v1_ID      );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 10, v2_ID      );
    // glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 11, xBasis_ID  );
    // glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 12, yBasis_ID  );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 13, posnArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 14, nghbrs_ID  );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 15, accel_ID   );

    // Stop talking to the buffer object?
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );

    // Cleanup
    printf( "Cleanup particle init ...\n" );
    delete_net( icosphr );
    free( v1_Arr );
    free( v2_Arr );
    free( mmbrArr );
}

void InitParticles( void ){
    // Get compute shader_ID info and allocate buffer space on the GPU
    // Author: Willem A. (Vlakkies) Schreüder

    // Get max workgroup size and count
    glGetIntegeri_v( GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &N_groups      );
    glGetIntegeri_v( GL_MAX_COMPUTE_WORK_GROUP_SIZE , 0, &workGroupSize );
    if( N_groups > 8192 ) N_groups = 8192;
    // _N_PARTICLES = workGroupSize * N_groups;

    // Initialize position buffer
    glGenBuffers( 1, &posnArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, posnArr_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, _N_PARTICLES * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize velocity buffer
    glGenBuffers( 1, &veloArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, veloArr_ID);
    glBufferData( GL_SHADER_STORAGE_BUFFER, _N_PARTICLES * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize color buffer
    glGenBuffers( 1, &colrArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, colrArr_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, _N_PARTICLES * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize member buffer
    glGenBuffers( 1, &mmbrArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, mmbrArr_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, _N_PARTICLES * sizeof( uint ), NULL, GL_STATIC_DRAW );

    // Initialize origin buffer
    glGenBuffers( 1, &origin_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, origin_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize v1 buffer
    glGenBuffers( 1, &v1_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, v1_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize v2 buffer
    glGenBuffers( 1, &v2_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, v2_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize X Basis buffer
    // glGenBuffers( 1, &xBasis_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, xBasis_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize Y Basis buffer
    // glGenBuffers( 1, &yBasis_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, yBasis_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize neighbors buffer
    glGenBuffers( 1, &nghbrs_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, nghbrs_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec3u ), NULL, GL_STATIC_DRAW );

    // Initialize acceleration buffer
    glGenBuffers( 1, &accel_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, accel_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );

    
    // Reset buffer positions
    ResetParticles();
}

////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

void cell_flow_interaction( void ){
    // Perturb and diffuse per-cell wind acceleration 
    uint ngbrDex;
    vec4f acl_i;
    vec4f acl_n;
    bool changed = false;
    for( uint i = 0; i < N_cells; ++i ){
        if( randf() < _DIFFUS_PROB ){
            ngbrDex = randu_range( 0, 2 );
            switch( ngbrDex ){
                case 0:
                    ngbrDex = ngbrArr[i].f0;
                    break;
                case 1:
                    ngbrDex = ngbrArr[i].f1;
                    break;
                case 2:
                    ngbrDex = ngbrArr[i].f2;
                    break;
                default:
                    printf( "THIS SHOULD NOT HAVE HAPPENED!" );
                    break;
            }
            acl_i = acclArr[i];
            acl_n = acclArr[ ngbrDex ];
            acclArr[i] /*---*/ = blend_vec4f( acl_n, _DIFFUS_RATE, acl_i, (1.0f-_DIFFUS_RATE) );
            acclArr[ ngbrDex ] = blend_vec4f( acl_i, _DIFFUS_RATE, acl_n, (1.0f-_DIFFUS_RATE) );
            if( norm_vec4f( acclArr[i] ) > _ACCEL_LIMIT ){  
                acclArr[i] = stretch_to_len_vec4f( acclArr[i], _ACCEL_LIMIT );  
            }
            if( norm_vec4f( acclArr[ ngbrDex ] ) > _ACCEL_LIMIT ){  
                acclArr[ ngbrDex ] = stretch_to_len_vec4f( acclArr[ ngbrDex ], _ACCEL_LIMIT );  
            }
            changed = true;
        }
        if( randf() < _PERTURB_PROB ){
            acclArr[i] = add_vec4f( acclArr[i], scale_vec4f( rand_vec4f(), randf_range( _ACCEL_MIN, _ACCEL_LIMIT ) ) );
            if( norm_vec4f( acclArr[i] ) > _ACCEL_LIMIT ){  
                acclArr[i] = stretch_to_len_vec4f( acclArr[i], _ACCEL_LIMIT );  
            }
            changed = true;
        }
    }
    
}

void tick(){
    // Background work

    // Launch compute shader
    glUseProgram( shaderDyna_ID );

    int id = glGetUniformLocation( shaderDyna_ID, "radius" );
    glUniform1f( id, _ATMOS_RADIUS );
    id = glGetUniformLocation( shaderDyna_ID, "speedLim" );
    glUniform1f( id, _SPEED_LIMIT );

    glDispatchComputeGroupSizeARB(
        N_groups, // ---- GLuint num_groups_x // FIXME: CHECK THAT THE # OF GROUPS IS CORRECT
        1, // ----------- GLuint num_groups_y
        1, // ----------- GLuint num_groups_z
        workGroupSize, // GLuint group_size_x
        1, // ----------- GLuint group_size_y
        1 // ------------ GLuint group_size_z
    );
    // glUseProgram(0);

    glUseProgram( shaderUpdt_ID );
    glDispatchComputeGroupSizeARB(
        N_groups, // ---- GLuint num_groups_x // FIXME: CHECK THAT THE # OF GROUPS IS CORRECT
        1, // ----------- GLuint num_groups_y
        1, // ----------- GLuint num_groups_z
        workGroupSize, // GLuint group_size_x
        1, // ----------- GLuint group_size_y
        1 // ------------ GLuint group_size_z
    );
    glUseProgram(0);

    //  Wait for compute shader
    glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );

    // printf( "tick, " );

    // Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void draw_particles( void ){
    // 1. Set particle size
    glPointSize(2);
    // 2. Vertex array
    glBindBuffer( GL_ARRAY_BUFFER, posnArr_ID );
    glVertexPointer( 4, GL_FLOAT, 0, (void*) 0 );
    // 3. Color array
    glBindBuffer( GL_ARRAY_BUFFER, colrArr_ID );
    glColorPointer( 4, GL_FLOAT, 0, (void*) 0 );
    // 4. Enable arrays used by DrawArrays
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_COLOR_ARRAY );
    // 5. Draw arrays
    glDrawArrays( GL_POINTS, 0, _N_PARTICLES );
    //  Disable arrays
    glDisableClientState( GL_VERTEX_ARRAY );
    glDisableClientState( GL_COLOR_ARRAY );
    //  Reset buffer
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
}


void display(){
    // Refresh display

    vec4f center = {0.0f,0.0f,0.0f,1.0f};
    vec4f sphClr = {0.0, 14.0f/255.0f, (214.0f-75.0f)/255.0f,1.0f};

    //  Erase the window and the depth buffer
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );

    glLoadIdentity();

    // Set view 
    look( cam );

    // //  Enable lighting
    // Lighting(dim*Cos(zh),dim,dim*Sin(zh) , 0.3,0.5,0.5);

    //  Draw sphere
    draw_sphere( center, _SPHERE_RADIUS, sphClr );

    // //  Disable lighting before particles
    // glDisable(GL_LIGHTING);

    

    //  Draw the particles
    draw_particles();

    //  Draw Axes
    // Axes( 500 );

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
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

////////// SIMULATION LOOP /////////////////////////////////////////////////////////////////////////

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){
    init_rand();
    // Initialize GLUT and process user parameters
    glutInit( &argc , argv );

    // Request window with size specified in pixels
    glutInitWindowSize( 900, 900 );

    // Create the window
    glutCreateWindow( "!!! PARTICLES !!!" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    // Request double buffered, true color window 
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    glEnable( GL_DEPTH_TEST );
    glDepthRange( 0.0f , 1.0f ); // WARNING: NOT IN THE EXAMPLE


    // //  Compute shader
    shaderDyna_ID = CreateShaderProgCompute( "shaders/prtclDyn.comp" );
    shaderUpdt_ID = CreateShaderProgCompute( "shaders/prtclMem.comp" );
    
    //  Initialize particles
    InitParticles();

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
    free( orgnArr );
    // free( v1_Arr  );
    free( xBasArr );
    free( yBasArr );
    free( acclArr );
    free( ngbrArr );

    printf( "\n### DONE ###\n\n" );
    
    //  Return code
    return 0;
}
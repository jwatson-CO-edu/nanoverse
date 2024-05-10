////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

/// View Settings ///
const float _SCALE /**/ = 750.0; // Scale Dimension
const int   _FOV_DEG    =  55; // - Field of view (for perspective)
const float _TARGET_FPS =  60.0f; // Desired framerate

/// Geometry ///
const float _SPHERE_RADIUS = 2.15f;
const float _ATMOS_RADIUS  = 2.25f;
const uint  _ICOS_SUBDIVID = 6;
const ulong N_cells /*--*/ = 20 * (_ICOS_SUBDIVID*(_ICOS_SUBDIVID+1)/2 + (_ICOS_SUBDIVID-1)*(_ICOS_SUBDIVID)/2);

/// Init ///
const uint _N_PARTICLES  = 1024;
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


////////// GLOBAL PROGRAM STATE ////////////////////////////////////////////////////////////////////
int workGroupSize;
int N_groups;

/// Particle per Row ///
uint posnArr_ID; //  Position buffer
uint veloArr_ID; //  Velocity buffer
uint colrArr_ID; //  Color buffer
uint mmbrArr_ID;

/// Cell per Row ///
uint origin_ID;
uint v1_ID;
uint v2_ID;
uint xBasis_ID;
uint yBasis_ID;
uint accel_ID;
uint nghbrs_ID;

///// CPU-Side State //////////////////////////////////////////////////////
vec4f* orgnArr = NULL;
vec4f* xBasArr = NULL;
vec4f* yBasArr = NULL;
vec4f* acclArr = NULL;


////////// GPU SETUP ///////////////////////////////////////////////////////////////////////////////

void ResetParticles(){
    // Write init data to buffers on the GPU
    // Adapted from code by Willem A. (Vlakkies) Schreüder

    float red_i, grn_i, blu_i;
    vec4f norm_i;
    vec4f *pos, *vel, *col, *trgtVf;
    vec4f* v1_Arr    = NULL;
    vec4f* v2_Arr    = NULL;
    TriNet* icosphr = create_icosphere_VFNA( _ATMOS_RADIUS, _ICOS_SUBDIVID );

    printf( "About to allocate CPU array memory ...\n" );
    orgnArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    xBasArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    yBasArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    acclArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );

    v1_Arr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    v2_Arr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );

    //  Reset position
    printf( "About to set bind buffer %u ...\n", posnArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, posnArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    printf( "About to map buffer %u ...\n", posnArr_ID );
    ErrCheck( "ResetParticles" );
    pos = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( vec4f ),
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    printf( "Sending particles to %p ...\n", pos );
    // Load init positions into buffer
    for (int i = 0; i < _N_PARTICLES; i++ ){
        pos[i].x = randf_range(    0,  100 );
        pos[i].y = randf_range( +400, +600 );
        pos[i].z = randf_range(  -50,  +50 );
        pos[i].w = 1;
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    //  Reset velocities
    printf( "About to set particle velocity, buffer %u ...\n", veloArr_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, veloArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    vel = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( vec4f ), 
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load init velocities into buffer
    for( int i = 0; i < _N_PARTICLES; i++ ){
        vel[i].x = randf_range( -10, +10 );
        vel[i].y = randf_range( -10, +10 );
        vel[i].z = randf_range( -10, +10 );
        vel[i].w = 0;
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
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    //  Reset origin
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

    //  Reset origin
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

    //  Reset origin
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

    //  Reset origin
    printf( "About to set cell X Basis, buffer %u ...\n", xBasis_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, xBasis_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        xBasArr[i] = trgtVf[i] = unit_vec4f( sub_vec4f( v1_Arr[i], orgnArr[i] ) );
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    //  Reset origin
    printf( "About to set cell y Basis, buffer %u ...\n", yBasis_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, yBasis_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    trgtVf = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                        GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        norm_i = get_CCW_tri_norm( orgnArr[i], v1_Arr[i], v2_Arr[i] );
        yBasArr[i] = trgtVf[i] = cross_vec4f( norm_i, xBasArr[i] );
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    

    // Associate buffer ID on GPU side with buffer ID on CPU side
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  4, posnArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  5, veloArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  6, colrArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  7, origin_ID  );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  8, v1_ID      );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER,  9, v2_ID      );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 10, xBasis_ID  );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 11, yBasis_ID  );

    // Stop talking to the buffer object?
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );

    // Cleanup
    delete_net( icosphr );
    free( v1_Arr );
    free( v2_Arr );
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

    // // Initialize member buffer
    // glGenBuffers( 1, &mmbrArr_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, mmbrArr_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, _N_PARTICLES * sizeof( uint ), NULL, GL_STATIC_DRAW );

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

    // Initialize X Basis buffer
    glGenBuffers( 1, &xBasis_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, xBasis_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // Initialize Y Basis buffer
    glGenBuffers( 1, &yBasis_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, yBasis_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize acceleration buffer
    // glGenBuffers( 1, &accel_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, accel_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize acceleration buffer
    // glGenBuffers( 1, &nghbrs_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, nghbrs_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec3u ), NULL, GL_STATIC_DRAW );


    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );

    

    // Reset buffer positions
    ResetParticles();
}

////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////

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
    glDepthRange( 0.0f , 1.0f ); // WARNING: NOT IN THE EXAMPLE


    // //  Compute shader
    // shader_ID = CreateShaderProgCompute( "shaders/06_Prtcl-Dyn.comp" );
    
    //  Initialize particles
    InitParticles();

    // //  Tell GLUT to call "display" when the scene should be drawn
    // glutDisplayFunc( display );

    // // Tell GLUT to call "idle" when there is nothing else to do
    // glutIdleFunc( tick );
    
    // //  Tell GLUT to call "reshape" when the window is resized
    // glutReshapeFunc( reshape );
    
    // //  Tell GLUT to call "special" when an arrow key is pressed
    // glutSpecialFunc( special );
    
    // //  Tell GLUT to call "key" when a key is pressed
    // glutKeyboardFunc( key );
    
    //  Pass control to GLUT so it can interact with the user
    // glutMainLoop();
    
    // // Free memory

    printf( "Cleanup!\n" );
    free( orgnArr );
    // free( v1_Arr  );
    free( xBasArr );
    free( yBasArr );
    free( acclArr );



    
    //  Return code
    return 0;
}
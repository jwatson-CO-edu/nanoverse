// gcc -std=c17 -O3 -Wall 05_atmos-GPU.c -lglut -lGLU -lGL -lm -o atmosGPU.out
#pragma GCC diagnostic ignored "-Wmissing-braces"

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "TriNet.h"
#include "OGL_Geo.h"
#include "OGL_Tools.h"


////////// VIEW SETTINGS & WINDOW STATE ////////////////////////////////////////////////////////////
const float _SCALE   = 10.0; //- Scale Dimension
float /*-*/ w2h /**/ =  0.0f; // Aspect ratio
int   /*-*/ fov /**/ = 55; // -- Field of view (for perspective)


static void Project(){
    // Set projection
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
    // NOTE: This function assumes that aspect rario will be computed by 'resize'
    
    //  Tell OpenGL we want to manipulate the projection matrix
    glMatrixMode( GL_PROJECTION );
    //  Undo previous transformations
    glLoadIdentity();
    
    gluPerspective( (double) fov , // -- Field of view angle, in degrees, in the y direction.
                    (double) w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
                    (double) _SCALE/4.0 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
                    (double) 4.0*_SCALE ); // Specifies the distance from the viewer to the far clipping plane (always positive).
    
    // Switch back to manipulating the model matrix
    glMatrixMode( GL_MODELVIEW );
    // Undo previous transformations
    glLoadIdentity();
}



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
Camera3D cam     = { {4.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
ulong    N_cells = 20 * (_ICOS_SUBDIVID*(_ICOS_SUBDIVID+1)/2 + (_ICOS_SUBDIVID-1)*(_ICOS_SUBDIVID)/2);
int /**/ shader_ID; // Shader program

///// GPU-Side State //////////////////////////////////////////////////////
int workGroupSize;
int N_groups;

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
uint mmbrArr_ID; // Color    array

///// CPU-Side State //////////////////////////////////////////////////////
vec4* orgnArr = NULL;
vec4* xBasArr = NULL;
vec4* yBasArr = NULL;
vec4* acclArr = NULL;


////////// GPU HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////

uint get_next_buffer_num( void ){
    // I guess these numbers start at 4?
    static uint bufNum = 3;
    ++bufNum;
    return bufNum;
}

///// Init & Get: `vec4` //////////////////////////////////////////////////

uint init_vec4_arr_at_GPU( unsigned long int N ){
    // Initialize a buffer at the GPU for `N` X `vec4`
    uint buf_ID = 0;
    glGenBuffers( 1, &buf_ID );
    printf( "\tAbout to map buffer %u ...\n", buf_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, buf_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N * sizeof( vec4 ), NULL, GL_STATIC_DRAW ); // GL_DYNAMIC_DRAW ); // GL_STATIC_DRAW );
    return buf_ID;
}

vec4* get_vec4_arr_from_buffer_obj( uint buf_ID, unsigned long int N ){
    // Allocate space on the GPU for an array of `vec4` of length `N`
    printf( "\tAbout to bind buffer ...\n" );
    vec4* rtnArr = NULL;
    // Reset position
    int res = glBindBuffer( GL_SHADER_STORAGE_BUFFER, buf_ID ); // Ask buffer object for a new buffer binding
    // Get pointer to buffer and cast as a struct array
    printf( "\tAbout to map buffer %u ...\n", buf_ID );
    rtnArr = (vec4*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N*sizeof( vec4 ),
                                               GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    printf( "\tAllocated %lu bytes on the GPU for `vec4`, Result: %i\n" , N*sizeof( vec4 ), res );
    return rtnArr;
}


///// Init & Get: `vec3uu` //////////////////////////////////////////////////

uint init_vec3uu_arr_at_GPU( unsigned long int N ){
    // Initialize a buffer at the GPU for `N` X `vec3uu`
    uint buf_ID = 0;
    glGenBuffers( 1, &buf_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, buf_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N * sizeof( vec3uu ), NULL, GL_STATIC_DRAW );
    return buf_ID;
}


vec3uu* get_vec3uu_arr_from_buffer_obj( uint buf_ID, unsigned long int N ){
    // Allocate space on the GPU for an array of `vec4` of length `N`
    vec3uu* rtnArr = NULL;
    // Reset position
    int res = glBindBuffer( GL_SHADER_STORAGE_BUFFER, buf_ID ); // Ask buffer object for a new buffer binding
    // Get pointer to buffer and cast as a struct array
    rtnArr = (vec3uu*) (long) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N*sizeof( vec3uu ),
                                                GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    printf( "Allocated %lu bytes on the GPU for `vec3uu`, Result: %i\n" , N*sizeof( vec3uu ), res );
    return rtnArr;
}


///// Init & Get: `uint` //////////////////////////////////////////////////

uint init_uint_arr_at_GPU( unsigned long int N ){
    // Initialize a buffer at the GPU for `N` X `uint`
    uint buf_ID = 0;
    glGenBuffers( 1, &buf_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, buf_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N * sizeof( uint ), NULL, GL_STATIC_DRAW );
    return buf_ID;
}


uint* get_uint_arr_from_buffer_obj( uint buf_ID, unsigned long int N ){
    // Allocate space on the GPU for an array of `vec4` of length `N`
    uint* rtnArr = NULL;
    // Reset position
    int res = glBindBuffer( GL_SHADER_STORAGE_BUFFER, buf_ID ); // Ask buffer object for a new buffer binding
    // Get pointer to buffer and cast as a struct array
    rtnArr = (uint*) (long) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N*sizeof( uint ),
                                              GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    printf( "Allocated %lu bytes on the GPU for `vec3uu`, Result: %i\n" , N*sizeof( uint ), res );
    return rtnArr;
}


///// Buffer Object Ops ///////////////////////////////////////////////////

void finish_buffer_assoc( uint buf_ID ){  
    // Unpoint the buffer object, Then associate a GPU buffer index with a CPU buffer index
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER );  
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, get_next_buffer_num(), buf_ID );
} 

void halt_buffer_obj( void ){  glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );  } // Stop talking to the buffer object?




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
    return rtnArr;
}



////////// ATMOSPHERE CONSTRUCTION & GPU MEMORY OPS ////////////////////////////////////////////////

void allocate_atmos_memory_at_GPU( void ){
    // Construct geo and move it to the GPU
    vec4* /*-*/ target   = NULL;
    vec3uu*     targetVU = NULL;
    uint* /*-*/ targetUI = NULL;
    printf( "About to create icosphere ...\n" );
    TriNet*     icosphr   = create_icosphere_VFNA( _ATMOS_RADIUS, _ICOS_SUBDIVID );
    vec4* /*-*/ v1_Arr    = NULL;
    uint* /*-*/ memberDex = NULL;
    vec4 /*--*/ norm_i    = {0.0f,0.0f,0.0f,1.0f};
    vec2 /*--*/ accl_i    = {0.0f,0.0f};
    float /*-*/ blu_i     = 0.0f;
    float /*-*/ grn_i     = 0.0f;

    // N_cells = (ulong) icosphr->Ntri;
    printf( "About to allocate CPU array memory ...\n" );
    orgnArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    v1_Arr  = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    xBasArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    yBasArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );
    acclArr = (vec4*) malloc( N_cells * sizeof( vec4 ) );

    if( !(orgnArr && v1_Arr && xBasArr && yBasArr && acclArr) ){
        printf( "!!! CPU malloc FAILURE !!!\n" );
    }else{
        printf( "CPU malloc SUCCESS\n" );
    }


    ///// Cell per Row ////////////////////////////////////////////////////

    /// Origin ///
    printf( "About to fetch origin buffer %u, %lu ...\n", origin_ID, N_cells );
    // Reset position
    int res = glBindBuffer( GL_SHADER_STORAGE_BUFFER, origin_ID ); // Ask buffer object for a new buffer binding
    // Get pointer to buffer and cast as a struct array
    printf( "\tAbout to map buffer %u ...\n", origin_ID );
    ErrCheck( "BEFORE `glMapBufferRange`" );
    target = (vec4*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells*sizeof( vec4 ),
                                               GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT );
    ErrCheck( "AFTER `glMapBufferRange`" );
    printf( "\tAllocated %lu bytes on the GPU for `vec4`, Result: %i\n" , N_cells*sizeof( vec4 ), res );
    printf( "Array Address: %p\n", target  );
    printf( "Array Address: %p\n", orgnArr );
    for( ulong i = 0; i < N_cells; ++i ){
        printf( "%f\n", orgnArr[i].x );
        printf( "%f\n", (*icosphr->V)[ (*icosphr->F)[i][0] ][0]  );
        printf( "%f\n", target[i].x  );
        
        orgnArr[i].x = target[i].x = (*icosphr->V)[ (*icosphr->F)[i][0] ][0];
        orgnArr[i].y = target[i].y = (*icosphr->V)[ (*icosphr->F)[i][0] ][1];
        orgnArr[i].z = target[i].z = (*icosphr->V)[ (*icosphr->F)[i][0] ][2];
        orgnArr[i].w = target[i].w = 1.0f;
    }
    finish_buffer_assoc( origin_ID );

    /// Vertex 1 ///
    printf( "About to fetch v1 buffer ...\n" );
    target = get_vec4_arr_from_buffer_obj( v1_ID, N_cells );
    for( ulong i = 0; i < N_cells; ++i ){
        v1_Arr[i].x = target[i].x = (*icosphr->V)[ (*icosphr->F)[i][1] ][0];
        v1_Arr[i].y = target[i].y = (*icosphr->V)[ (*icosphr->F)[i][1] ][1];
        v1_Arr[i].z = target[i].z = (*icosphr->V)[ (*icosphr->F)[i][1] ][2];
        v1_Arr[i].w = target[i].w = 1.0f;
    }
    finish_buffer_assoc( v1_ID );

    /// Vertex 2 ///
    target = get_vec4_arr_from_buffer_obj( v2_ID, N_cells );
    for( ulong i = 0; i < N_cells; ++i ){
        target[i].x = (*icosphr->V)[ (*icosphr->F)[i][2] ][0];
        target[i].y = (*icosphr->V)[ (*icosphr->F)[i][2] ][1];
        target[i].z = (*icosphr->V)[ (*icosphr->F)[i][2] ][2];
        target[i].w = 1.0f;
    }
    finish_buffer_assoc( v2_ID );

    /// X Basis ///
    target = get_vec4_arr_from_buffer_obj( xBasis_ID, N_cells );
    for( uint i = 0; i < N_cells; ++i ){
        xBasArr[i] = target[i] = unit_vec4( sub_vec4( v1_Arr[i], orgnArr[i] ) );
    }
    finish_buffer_assoc( xBasis_ID );

    /// Y Basis ///
    target = get_vec4_arr_from_buffer_obj( yBasis_ID, N_cells );
    for( uint i = 0; i < N_cells; ++i ){
        norm_i.x = (*icosphr->N)[i][0];
        norm_i.y = (*icosphr->N)[i][1];
        norm_i.z = (*icosphr->N)[i][2];
        yBasArr[i] = target[i] = unit_vec4( cross_vec4( norm_i, xBasArr[i] ) );
    }
    finish_buffer_assoc( yBasis_ID );

    /// Acceleration ///
    target = get_vec4_arr_from_buffer_obj( accel_ID, N_cells );
    for( uint i = 0; i < N_cells; ++i ){
        accl_i.x = randf_range( -_ACCEL_LIMIT, +_ACCEL_LIMIT );
        accl_i.y = randf_range( -_ACCEL_LIMIT, +_ACCEL_LIMIT );
        if( norm_vec2( accl_i ) < _ACCEL_MIN ){  accl_i = stretch_vec2_to_len( accl_i, _ACCEL_MIN );  }
        acclArr[i] = target[i] = lift_vec_2D_to_3D( accl_i, xBasArr[i], yBasArr[i] );
    }
    finish_buffer_assoc( accel_ID );

    /// Neighbors ///
    targetVU = get_vec3uu_arr_from_buffer_obj( nghbrs_ID, N_cells );
    for( uint i = 0; i < N_cells; ++i ){
        targetVU[i].f0 = (*icosphr->A)[i][0];
        targetVU[i].f1 = (*icosphr->A)[i][1];
        targetVU[i].f2 = (*icosphr->A)[i][2];
    }
    finish_buffer_assoc( nghbrs_ID );


    ///// Particle per Row ////////////////////////////////////////////////

    /// Position ///
    target    = get_vec4_arr_from_buffer_obj( posnArr_ID, _N_PARTICLES );
    memberDex = distribute_particles_init( 
        target, orgnArr, xBasArr, yBasArr, diff_vec4( orgnArr[0], v1_Arr[0] ), 200, _N_PARTICLES/1000 
    );
    finish_buffer_assoc( posnArr_ID );

    /// Membership ///
    targetUI = get_uint_arr_from_buffer_obj( mmbrArr_ID, _N_PARTICLES );
    for( uint i = 0; i < _N_PARTICLES; ++i ){  targetUI[i] = memberDex[i];  }
    finish_buffer_assoc( mmbrArr_ID );

    /// Velocity ///
    /*-*/ target    = get_vec4_arr_from_buffer_obj( veloArr_ID, _N_PARTICLES );
    float factorMin = 2.0f;
    float factorMax = 4.0f;
    for( uint i = 0; i < _N_PARTICLES; ++i ){
        target[i] = scale_vec4( acclArr[ memberDex[i] ], randf_range( factorMin, factorMax )  );
    }
    finish_buffer_assoc( veloArr_ID );

    /// Color ///
    target = get_vec4_arr_from_buffer_obj( colrArr_ID, _N_PARTICLES );
    for( uint i = 0; i < _N_PARTICLES; ++i ){
        grn_i = randf_range( 0.25f, 1.00f );
        blu_i = randf_range( 0.25f, 1.00f );
        target[i].r = (grn_i + blu_i)/2.0f;
        target[i].g = grn_i;
        target[i].b = blu_i;
        target[i].a = 1.0f;
    }
    finish_buffer_assoc( colrArr_ID );

    /// Cleanup ///
    halt_buffer_obj();
    delete_net( icosphr );
    free( v1_Arr    );
    free( memberDex );
}


void init_atmos_memory_and_workers_at_GPU( void ){
    // Init worker groups and all arrays
    // Get max workgroup size and count
    printf( "About to init workers ...\n" );
    glGetIntegeri_v( GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &N_groups      );
    glGetIntegeri_v( GL_MAX_COMPUTE_WORK_GROUP_SIZE , 0, &workGroupSize );
    if( N_groups > 8192 ) N_groups = 8192;
    ///// Cell per Row /////
    printf( "About to init cell memory at GPU ...\n" );
    glGenBuffers( 1, &origin_ID );
    printf( "\tAbout to map buffer %u ...\n", origin_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, origin_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4 ), NULL, GL_STATIC_DRAW );
    v1_ID     = init_vec4_arr_at_GPU( N_cells );
    v2_ID     = init_vec4_arr_at_GPU( N_cells );
    xBasis_ID = init_vec4_arr_at_GPU( N_cells );
    yBasis_ID = init_vec4_arr_at_GPU( N_cells );
    accel_ID  = init_vec4_arr_at_GPU( N_cells );
    nghbrs_ID = init_vec3uu_arr_at_GPU( N_cells );
    ///// Particle per Row /////
    printf( "About to init particle memory at GPU ...\n" );
    posnArr_ID = init_vec4_arr_at_GPU( _N_PARTICLES );
    mmbrArr_ID = init_uint_arr_at_GPU( _N_PARTICLES );
    veloArr_ID = init_vec4_arr_at_GPU( _N_PARTICLES );
    colrArr_ID = init_vec4_arr_at_GPU( _N_PARTICLES );
    ///// Stop Buffer Init /////
    halt_buffer_obj();
    ///// Allocate /////
    printf( "About to allocate GPU array memory ...\n" );
    allocate_atmos_memory_at_GPU();
}


////////// RENDERING ///////////////////////////////////////////////////////////////////////////////

void draw_particles( void ){
    // Render all particles after the compute shader_ID has moved them
    // Author: Willem A. (Vlakkies) Schreüder
    // Set particle size
    glPointSize(1);
    // Vertex array
    glBindBuffer( GL_ARRAY_BUFFER, posnArr_ID );
    glVertexPointer( 4, GL_FLOAT, 0, (void*) 0 );
    // Color array
    glBindBuffer( GL_ARRAY_BUFFER, colrArr_ID );
    glColorPointer( 4, GL_FLOAT, 0, (void*) 0 );
    // Enable arrays used by DrawArrays
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_COLOR_ARRAY  );
    // Draw arrays
    glDrawArrays( GL_POINTS, 0, _N_PARTICLES );
    // Disable arrays
    glDisableClientState( GL_VERTEX_ARRAY );
    glDisableClientState( GL_COLOR_ARRAY  );
    // Reset buffer
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
}





void display(){
    // Draw one frame
    // Adapted from work by Willem A. (Vlakkies) Schreüder

    // const float SphereY = -500;
    // const float SphereR = +600;
    // int /*---*/ id = 0;

    // Erase the window and the depth buffer
    glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    
    // Set view 
    look( cam );
    
    // // Enable lighting
    // Lighting( dim * Cos( zh ), dim, dim * Sin(zh), 0.3, 0.5, 0.5 );

    // // Draw sphere
    // SetColor( 0.8, 0.8, 0 );
    // glPushMatrix();
    // glTranslatef( 0, SphereY, 0 );
    // glRotatef( -90, 1, 0, 0 );
    // glScaled( SphereR, SphereR, SphereR );
    // SolidSphere( 32 );
    // glPopMatrix();


    ///// Compute Shader //////////////////////////////////////////////////

    // // Disable lighting before particles
    // glDisable( GL_LIGHTING );

    // // Launch compute `shader_ID`
    // glUseProgram( shader_ID ); // GPU state points to the compute shader

    // // Fetch ID of var that holds sphere center location, and set the location
    // id = glGetUniformLocation( shader_ID, "xyz" );
    // glUniform3f( id, 0, SphereY, 0 );

    // // Fetch ID of var that holds sphere radius, and set the radius
    // id = glGetUniformLocation( shader_ID, "dim" );
    // glUniform1f( id, SphereR );
    
    // // Launch workers to perform tasks, Each task is defined by the shader program
    // glDispatchComputeGroupSizeARB( 
    //     // Array of Worker Groups //
    //     _N_PARTICLES/workGroupSize, // Number of groups, Dim 0
    //     1, // ----------------- Number of groups, Dim 1
    //     1, // ----------------- Number of groups, Dim 2
    //     // Array of Workers within each Group //
    //     workGroupSize, // Group size, Dim 0
    //     1, // ----------- Group size, Dim 1
    //     1 // ------------ Group size, Dim 2
    // );
    // glUseProgram(0); // GPU state no longer points to a shader program

    // //  Wait for compute shader_ID
    // glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );

    //  Draw the particles
    draw_particles();

    // //  Draw Axes
    // Axes(500);

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
    Print( "FPS %f", heartbeat_FPS( 60.0f ) );
    
    // Check errors
    ErrCheck("display");

    // Render the scene and make it visible: Flush and swap
    glFlush();
    glutSwapBuffers();
}



////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////

void reshape( int width , int height ){
    // GLUT calls this routine when the window is resized
    // Calc the aspect ratio: width to the height of the window
    w2h = ( height > 0 ) ? (float) width / height : 1;
    // Set the viewport to the entire window
    glViewport( 0 , 0 , width , height );
    // Set projection
    Project();
}

////////// SIMULATION LOOP /////////////////////////////////////////////////////////////////////////

void tick( void ){
    // Simulation updates in between repaints
    // tick_atmos( simpleAtmos );

    //  Tell GLUT it is necessary to redisplay the scene
    glutPostRedisplay();
}



////////// SHADER LOADING UTILS ////////////////////////////////////////////////////////////////////

void Fatal( const char* format, ... ){
    // Scream and run
    // // Author: Willem A. (Vlakkies) Schreüder  
    va_list args;
    va_start(args,format);
    vfprintf(stderr,format,args);
    va_end(args);
    exit(1);
}

static char* ReadText( const char *fileName ){
    // Read and return the contents of a text file
    // Author: Willem A. (Vlakkies) Schreüder  
    int   n;
    char* buffer;
    //  Open fileName
    FILE* f = fopen( fileName, "rb" );
    if (!f) Fatal( "Cannot open text fileName %s\n", fileName );
    //  Seek to end to determine size, then rewind
    fseek( f, 0, SEEK_END );
    n = ftell(f);
    rewind(f);
    //  Allocate memory for the whole fileName
    buffer = (char*) malloc( n+1 );
    if( !buffer ) Fatal( "Cannot allocate %d bytes for text fileName %s\n", n+1, fileName );
    //  Snarf the fileName
    if( fread( buffer, n, 1, f ) != 1 ) Fatal( "Cannot read %d bytes for text fileName %s\n", n, fileName );
    buffer[n] = 0;
    //  Close and return
    fclose(f);
    return buffer;
}


static void PrintShaderLog( int obj, const char* fileName ){
    // Get compilation output of the shader `obj`
    // Author: Willem A. (Vlakkies) Schreüder  
    int len = 0;
    glGetShaderiv( obj, GL_INFO_LOG_LENGTH, &len );
    if( len>1 ){
        int   n /**/ = 0;
        char* buffer = (char*) malloc( len );
        if( !buffer ) Fatal( "Cannot allocate %d bytes of text for shader log\n", len );
        glGetShaderInfoLog( obj, len, &n, buffer );
        fprintf( stderr, "%s:\n%s\n", fileName, buffer );
        free( buffer );
    }else{
        printf( "%s: No shader log contents.\n", fileName );
    }
    glGetShaderiv( obj, GL_COMPILE_STATUS, &len );
    if (!len) Fatal( "Error compiling %s\n", fileName );
}


void CreateShader( int progNum, const GLenum type, const char* fileName ){
    // Create the shader
    // Author: Willem A. (Vlakkies) Schreüder  
    int shader = glCreateShader( type );
    // Load source code from fileName
    char* sourceText = ReadText( fileName );
    glShaderSource( shader, 1, (const char**)&sourceText, NULL );
    free( sourceText );
    // Compile the shader
    glCompileShader( shader );
    // Check for errors
    PrintShaderLog( shader, fileName );
    // Attach to shader program
    glAttachShader( progNum, shader );
}


void PrintProgramLog( int obj ){
    // Get linking output for shader `obj`
    // Author: Willem A. (Vlakkies) Schreüder  
    int len = 0;
    glGetProgramiv( obj, GL_INFO_LOG_LENGTH, &len );
    if( len > 1 ){
        int   n /**/ = 0;
        char* buffer = (char*) malloc( len );
        if (!buffer) Fatal("Cannot allocate %d bytes of text for program log\n",len);
        glGetProgramInfoLog( obj, len, &n, buffer );
        fprintf( stderr, "%s\n", buffer );
    }else{
        printf( "Object %i: No program log contents.\n", obj );
    }
    glGetProgramiv( obj, GL_LINK_STATUS, &len );
    if( !len )  Fatal( "Error linking program\n" );
}


int CreateShaderProgCompute( char* fileName ){
   // Create compute shader program
   // Author: Willem A. (Vlakkies) Schreüder  
   int progNum = glCreateProgram();
   // Create and compile compute shader
   CreateShader( progNum, GL_COMPUTE_SHADER, fileName );
   // Link program
   glLinkProgram( progNum );
   // Check for errors
   PrintProgramLog( progNum );
   // Return name
   return progNum;
}


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main( int argc , char* argv[] ){
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


    
    //  Compute shader
    // printf( "About to compile shader ...\n" );
    shader_ID = CreateShaderProgCompute( "shaders/06_Prtcl-Dyn.comp" );
    
    //  Initialize particles
    printf( "About to init shader memory ...\n" );
    init_atmos_memory_and_workers_at_GPU();

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
    
    //  Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    // // Free memory
    // delete_net( icos );
    // delete_atmos( simpleAtmos );

    free( orgnArr );
    free( xBasArr );
    free( yBasArr );
    free( acclArr );

    //  Return code
    return 0;
}
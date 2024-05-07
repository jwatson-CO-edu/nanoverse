// gcc -std=c17 -O3 -Wall 05_atmos-GPU.c -lglut -lGLU -lGL -lm -o atmosGPU.out

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "ZZ_Utils.h"


////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////

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


int /*----*/ shader_ID; //  Shader program

int    th  =    0; // Azimuth of view angle
int    ph  =    0; // Elevation of view angle
int    zh  =   30; // Light angle
double asp =    1; // Aspect ratio
double dim = 1000; // Size of world




////////// GPU SETUP ///////////////////////////////////////////////////////////////////////////////

void ResetParticles(){
    // Write init data to buffers on the GPU
    // Author: Willem A. (Vlakkies) Schreüder

    vec4f *pos, *vel, *col;
    vec4f* /*-*/ v1_Arr    = NULL;
    // TriNet*     icosphr   = create_icosphere_VFNA( _ATMOS_RADIUS, _ICOS_SUBDIVID );

    printf( "About to allocate CPU array memory ...\n" );
    orgnArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    v1_Arr  = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    xBasArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    yBasArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );
    acclArr = (vec4f*) malloc( N_cells * sizeof( vec4f ) );

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
    printf( "About to set particle velocity ...\n" );
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
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, colrArr_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    col = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, _N_PARTICLES * sizeof( vec4f ), 
                                    GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( int i = 0; i < _N_PARTICLES; i++ ){
        col[i].r = randf_range( 0.1, 1.0 );
        col[i].g = randf_range( 0.1, 1.0 );
        col[i].b = randf_range( 0.1, 1.0 );
        col[i].a = 1.;
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object


    //  Reset origin
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, origin_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    col = (vec4f*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_cells * sizeof( vec4f ), 
                                     GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( ulong i = 0; i < N_cells; ++i ){
        // printf( "%f\n", orgnArr[i].x );
        // printf( "%f\n", icosphr->V[ icosphr->F[i].v0 ].x  );
        // printf( "%f\n", col[i].x  );
        // orgnArr[i] = col[i] = icosphr->V[ icosphr->F[i].v0 ];
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    // Associate buffer ID on GPU side with buffer ID on CPU side
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 4, posnArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 5, veloArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 6, colrArr_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 7, origin_ID  );

    // Stop talking to the buffer object?
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );
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

    // // Initialize origin buffer
    // glGenBuffers( 1, &origin_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, origin_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize v1 buffer
    // glGenBuffers( 1, &v1_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, v1_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize v2 buffer
    // glGenBuffers( 1, &v2_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, v2_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize X Basis buffer
    // glGenBuffers( 1, &xBasis_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, xBasis_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

    // // Initialize Y Basis buffer
    // glGenBuffers( 1, &yBasis_ID );
    // glBindBuffer( GL_SHADER_STORAGE_BUFFER, yBasis_ID );
    // glBufferData( GL_SHADER_STORAGE_BUFFER, N_cells * sizeof( vec4f ), NULL, GL_STATIC_DRAW );

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

void DrawParticles( void ){
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

float lastTime = 0.0f;

float heartbeat( float targetFPS ){
    // Attempt to maintain framerate no greater than target. (Period is rounded down to next ms)
    float currTime   = 0.0f;
    float framTime   = 0.0f;
    float target_ms  = 1000.0f / targetFPS;
    static float FPS = 0.0f;
    framTime = (float) glutGet( GLUT_ELAPSED_TIME ) - lastTime;
    if( framTime < target_ms ){ sleep_ms( (long) (target_ms - framTime) );  }
    currTime = (float) glutGet( GLUT_ELAPSED_TIME );
    FPS = (1000.0f / (currTime - lastTime)) * 0.125f + FPS * 0.875f; // Filter for readable number
    lastTime = currTime;
    return FPS;
}

void display(){
    // Draw one frame
    // Adapted from work by Willem A. (Vlakkies) Schreüder

    const float SphereY = -500;
    const float SphereR = +600;
    int /*---*/ id = 0;

    // Erase the window and the depth buffer
    glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    
    // Set eye position
    View( th, ph, 55, dim );
    
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

    // Disable lighting before particles
    glDisable( GL_LIGHTING );

    // Launch compute `shader_ID`
    glUseProgram( shader_ID ); // GPU state points to the compute shader

    // Fetch ID of var that holds sphere center location, and set the location
    id = glGetUniformLocation( shader_ID, "xyz" );
    glUniform3f( id, 0, SphereY, 0 );

    // Fetch ID of var that holds sphere radius, and set the radius
    id = glGetUniformLocation( shader_ID, "dim" );
    glUniform1f( id, SphereR );
    
    // Launch workers to perform tasks, Each task is defined by the shader program
    glDispatchComputeGroupSizeARB( 
        // Array of Worker Groups //
        _N_PARTICLES/workGroupSize, // Number of groups, Dim 0
        1, // ----------------- Number of groups, Dim 1
        1, // ----------------- Number of groups, Dim 2
        // Array of Workers within each Group //
        workGroupSize, // Group size, Dim 0
        1, // ----------- Group size, Dim 1
        1 // ------------ Group size, Dim 2
    );
    glUseProgram(0); // GPU state no longer points to a shader program

    //  Wait for compute shader_ID
    glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );

    //  Draw the particles
    DrawParticles();

    // //  Draw Axes
    // Axes(500);

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
    Print( "%d,%f FPS=%d Dim=%.1f Size=%d Count=%d N=%d", 
           th, ph, heartbeat( 60.0 ), dim, workGroupSize, N_groups, _N_PARTICLES );
    
    // Check errors
    ErrCheck("display");

    // Render the scene and make it visible: Flush and swap
    glFlush();
    glutSwapBuffers();
}

void Projection( float fov, float asp, float dim ){
   // Set projection matrix
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   // Perspective transformation
   gluPerspective( fov, asp, dim/16, 16*dim );
   // Reset modelview
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////

void reshape( int width , int height ){
    // GLUT calls this routine when the window is resized
    // Calc the aspect ratio: width to the height of the window
    asp = ( height > 0 ) ? (float) width / height : 1;
    // Set the viewport to the entire window
    glViewport( 0 , 0 , width , height );
    // Set projection
    Projection( 55, asp, dim );
}

////////// SIMULATION LOOP /////////////////////////////////////////////////////////////////////////

void tick(){
    // Simulation updates in between repaints
    // tick_atmos( simpleAtmos );

    //  Tell GLUT it is necessary to redisplay the scene
    glutPostRedisplay();
}


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


    //  Compute shader
    shader_ID = CreateShaderProgCompute( "shaders/06_Prtcl-Dyn.comp" );
    
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
    
    //  Pass control to GLUT so it can interact with the user
    glutMainLoop();
    
    // // Free memory
    // delete_net( icos );
    // delete_atmos( simpleAtmos );
    
    //  Return code
    return 0;
}
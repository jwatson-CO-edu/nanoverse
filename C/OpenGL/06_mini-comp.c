////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "AdvClass.h"



////////// GLOBAL PROGRAM STATE ////////////////////////////////////////////////////////////////////
int workGroupSize;
int N_groups;
int N_prt;

unsigned int posbuf_ID; //  Position buffer
unsigned int velbuf_ID; //  Velocity buffer
unsigned int colbuf_ID; //  Color buffer
int /*----*/ shader_ID; //  Shader program

int    th  =    0; // Azimuth of view angle
int    ph  =    0; // Elevation of view angle
int    zh  =   30; // Light angle
double asp =    1; // Aspect ratio
double dim = 1000; // Size of world

////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////

typedef struct{
  union{ float x; float r; };
  union{ float y; float g; };
  union{ float z; float b; };
  union{ float w; float a; };
} vec4;



////////// GPU SETUP ///////////////////////////////////////////////////////////////////////////////

void ResetParticles(){
    // Write init data to buffers on the GPU
    // Author: Willem A. (Vlakkies) Schreüder

    vec4 *pos, *vel, *col;

    //  Reset position
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, posbuf_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    pos = (vec4*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_prt * sizeof( vec4 ),
                                    GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load init positions into buffer
    for (int i = 0; i < N_prt; i++ ){
        pos[i].x = frand(    0,  100 );
        pos[i].y = frand( +400, +600 );
        pos[i].z = frand(  -50,  +50 );
        pos[i].w = 1;
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    //  Reset velocities
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, velbuf_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    vel = (vec4*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_prt * sizeof( vec4 ), 
                                    GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load init velocities into buffer
    for( int i = 0; i < N_prt; i++ ){
        vel[i].x = frand( -10, +10 );
        vel[i].y = frand( -10, +10 );
        vel[i].z = frand( -10, +10 );
        vel[i].w = 0;
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    //  Reset colors
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, colbuf_ID ); // Set buffer object to point to this ID, for writing
    // Get pointer to buffer and cast as a struct array
    col = (vec4*) glMapBufferRange( GL_SHADER_STORAGE_BUFFER, 0, N_prt * sizeof( vec4 ), 
                                    GL_MAP_WRITE_BIT|GL_MAP_INVALIDATE_BUFFER_BIT      );
    // Load colors into buffer
    for( int i = 0; i < N_prt; i++ ){
        col[i].r = frand( 0.1, 1.0 );
        col[i].g = frand( 0.1, 1.0 );
        col[i].b = frand( 0.1, 1.0 );
        col[i].a = 1.;
    }
    glUnmapBuffer( GL_SHADER_STORAGE_BUFFER ); // Release buffer object

    // Associate buffer ID on GPU side with buffer ID on CPU side
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 4, posbuf_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 5, velbuf_ID );
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 6, colbuf_ID );

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
    N_prt = workGroupSize * N_groups;

    // Initialize position buffer
    glGenBuffers( 1, &posbuf_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, posbuf_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_prt * sizeof( vec4 ), NULL, GL_STATIC_DRAW );

    // Initialize velocity buffer
    glGenBuffers( 1, &velbuf_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, velbuf_ID);
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_prt * sizeof( vec4 ), NULL, GL_STATIC_DRAW );

    // Initialize color buffer
    glGenBuffers( 1, &colbuf_ID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, colbuf_ID );
    glBufferData( GL_SHADER_STORAGE_BUFFER, N_prt * sizeof( vec4 ), NULL, GL_STATIC_DRAW );
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
    glBindBuffer( GL_ARRAY_BUFFER, posbuf_ID );
    glVertexPointer( 4, GL_FLOAT, 0, (void*) 0 );
    // Color array
    glBindBuffer( GL_ARRAY_BUFFER, colbuf_ID );
    glColorPointer( 4, GL_FLOAT, 0, (void*) 0 );
    // Enable arrays used by DrawArrays
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_COLOR_ARRAY  );
    // Draw arrays
    glDrawArrays( GL_POINTS, 0, N_prt );
    // Disable arrays
    glDisableClientState( GL_VERTEX_ARRAY );
    glDisableClientState( GL_COLOR_ARRAY  );
    // Reset buffer
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
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
        N_prt/workGroupSize, // Number of groups, Dim 0
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
    DrawPart();

    // //  Draw Axes
    // Axes(500);

    //  Display parameters
    glDisable( GL_DEPTH_TEST );
    glWindowPos2i( 5, 5 );
    Print( "%d,%d FPS=%d Dim=%.1f Size=%d Count=%d N=%d", 
           th, ph, FramesPerSecond(), dim, workGroupSize, N_groups, N_prt );
    
    // Check errors
    ErrCheck("display");

    // Render the scene and make it visible: Flush and swap
    glFlush();
    glutSwapBuffers();
}
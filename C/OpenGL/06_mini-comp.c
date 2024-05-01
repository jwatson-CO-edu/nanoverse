////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "AdvClass.h"

////////// PROGRAM VARS ////////////////////////////////////////////////////////////////////////////
int workGroupSize;
int N_groups;
int N_prt;

unsigned int posbuf_ID; //  Position buffer
unsigned int velbuf_ID; //  Velocity buffer
unsigned int colbuf_ID; //  Color buffer

typedef struct {
  union{ float x; float r; };
  union{ float y; float g; };
  union{ float z; float b; };
  union{ float w; float a; };
} vec4;

void ResetPart(){
    // Write init data to buffers on the GPU

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


void InitPart( void ){
    // Get compute shader info and allocate buffer space on the GPU

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
    ResetPart();
}
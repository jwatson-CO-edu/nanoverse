#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" // UNKNOWN MAGIC

#ifndef ADVCLASS_H // This pattern is to prevent symbols to be loaded multiple times
#define ADVCLASS_H // from multiple imports

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>


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


void Lighting( float x, float y, float z, float ambient, float diffuse, float specular ){
    // Set params for OGL Light 0 and Activate
    // Translate intensity to color vectors
    float Ambient[]  = {ambient , ambient , ambient , 1.0};
    float Diffuse[]  = {diffuse , diffuse , diffuse , 1.0};
    float Specular[] = {specular, specular, specular, 1.0};
    // Light position
    float Position[] = {x,y,z,1};

    // Draw light position as ball before enabling lighting
    SetColor( 1, 1, 1 );
    Sphere( x, y, z, 0.1, 0, 8, 0 );

    //  OpenGL should normalize normal vectors
    glEnable( GL_NORMALIZE );
    //  Enable lighting
    glEnable( GL_LIGHTING );

    //  Enable light 0
    glEnable(GL_LIGHT0);
    //  Set ambient, diffuse, specular components and position of light 0
    glLightfv(GL_LIGHT0,GL_AMBIENT ,Ambient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE ,Diffuse);
    glLightfv(GL_LIGHT0,GL_SPECULAR,Specular);
    glLightfv(GL_LIGHT0,GL_POSITION,Position);
}


void View( float th, float ph, float fov, float dim ){
    //  Set ModelView matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    //  Perspective - set eye position
    float Ex = -2*dim*Sin(th)*Cos(ph);
    float Ey = +2*dim        *Sin(ph);
    float Ez = +2*dim*Cos(th)*Cos(ph);
    gluLookAt( Ex, Ey, Ez, 0, 0, 0, 0, Cos( ph ), 0 );
}

static int fps   = 0,
/*------*/ sec0  = 0,
/*------*/ count = 0;

int FramesPerSecond( void ){
   int sec = glutGet( GLUT_ELAPSED_TIME );
   if( sec != sec0 ){
      sec0  = sec;
      fps   = count;
      count = 0;
   }
   count++;
   return fps;
}


#endif
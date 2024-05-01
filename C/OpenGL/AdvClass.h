#pragma GCC diagnostic ignored "-Wimplicit-function-declaration" // UNKNOWN MAGIC

#ifndef ADVCLASS_H // This pattern is to prevent symbols to be loaded multiple times
#define ADVCLASS_H // from multiple imports

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/glu.h>
#include <GL/gl.h>

void Fatal( const char* format, ... ){
   va_list args;
   va_start(args,format);
   vfprintf(stderr,format,args);
   va_end(args);
   exit(1);
}

static char* ReadText(const char *fileName){
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
   int len = 0;
   glGetShaderiv( obj, GL_INFO_LOG_LENGTH, &len );
   if( len>1 ){
      int   n /**/ = 0;
      char* buffer = (char *)malloc(len);
      if( !buffer ) Fatal( "Cannot allocate %d bytes of text for shader log\n", len );
      glGetShaderInfoLog( obj, len, &n, buffer );
      fprintf( stderr, "%s:\n%s\n", fileName, buffer );
      free( buffer );
   }
   glGetShaderiv( obj, GL_COMPILE_STATUS, &len );
   if (!len) Fatal( "Error compiling %s\n", fileName );
}


void CreateShader( int progNum, const GLenum type, const char* fileName ){
   //  Create the shader
   int shader = glCreateShader( type );
   //  Load source code from fileName
   char* sourceText = ReadText( fileName );
   glShaderSource( shader, 1, (const char**)&sourceText, NULL );
   free( sourceText );
   //  Compile the shader
   glCompileShader( shader );
   //  Check for errors
   PrintShaderLog( shader, fileName );
   //  Attach to shader program
   glAttachShader( progNum, shader );
}

void PrintProgramLog( int obj ){
   int len = 0;
   glGetProgramiv( obj, GL_INFO_LOG_LENGTH, &len );
   if( len>1 ){
      int   n /**/ = 0;
      char* buffer = (char*) malloc( len );
      if (!buffer) Fatal("Cannot allocate %d bytes of text for program log\n",len);
      glGetProgramInfoLog(obj,len,&n,buffer);
      fprintf(stderr,"%s\n",buffer);
   }
   glGetProgramiv( obj, GL_LINK_STATUS, &len );
   if (!len) Fatal( "Error linking program\n" );
}

int CreateShaderProgCompute( char* fileName ){
   //  Create program
   int progNum = glCreateProgram();
   //  Create and compile compute shader
   CreateShader( progNum, GL_COMPUTE_SHADER, fileName );
   //  Link program
   glLinkProgram( progNum );
   //  Check for errors
   PrintProgramLog( progNum );
   //  Return name
   return progNum;
}

#endif
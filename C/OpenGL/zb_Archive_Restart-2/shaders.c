////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"



////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////

char* ReadText( char *file ){
    // Read the contents of a text file
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    char* buffer;
    // 1. Open file
    FILE* f = fopen( file, "rt" );
    if( !f )  Fatal( "Cannot open text file %s\n", file );
    // 2. Seek to end to determine size, then rewind
    fseek( f, 0, SEEK_END );
    int n = ftell( f );
    rewind( f );
    // 3. Allocate memory for the whole file
    buffer = (char*)malloc(n+1);
    if( !buffer )  Fatal( "Cannot allocate %d bytes for text file %s\n", n+1, file );
    // 4. Snarf the file
    if( fread( buffer, n, 1, f ) != 1 )  Fatal( "Cannot read %d bytes for text file %s\n", n, file );
    buffer[n] = 0;
    // N. Close and return
    fclose(f);
    return buffer;
}



////////// SHADER STATUS ///////////////////////////////////////////////////////////////////////////

void PrintShaderLog( int obj, char* file ){
    // Attempt to retrieve compilation status of the shader program
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    int len = 0;
    glGetShaderiv( obj, GL_INFO_LOG_LENGTH, &len );
    if (len>1)
    {
    int n=0;
    char* buffer = (char *)malloc(len);
    if (!buffer) Fatal("Cannot allocate %d bytes of text for shader log\n",len);
    glGetShaderInfoLog(obj,len,&n,buffer);
    fprintf(stderr,"%s:\n%s\n",file,buffer);
    free(buffer);
    }
    glGetShaderiv(obj,GL_COMPILE_STATUS,&len);
    if (!len) Fatal("Error compiling %s\n",file);
}


void PrintProgramLog( int obj ){
    // Print Program Log
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    int len = 0;
    glGetProgramiv( obj, GL_INFO_LOG_LENGTH, &len );
    if (len>1){
        int   n /**/ = 0;
        char* buffer = (char *) malloc( len );
        if( !buffer )  Fatal( "Cannot allocate %d bytes of text for program log\n", len );
        glGetProgramInfoLog( obj, len, &n, buffer );
        fprintf( stderr, "%s\n", buffer );
    }
    glGetProgramiv( obj, GL_LINK_STATUS, &len );
    if( !len )  Fatal( "Error linking program\n" );
}


////////// SHADER INSTANTIATION ////////////////////////////////////////////////////////////////////

int CreateShader( GLenum type,char* file ){
   // Create the shader program and return its ID
   // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
   // 0. Init shader at next index
   int shader = glCreateShader( type );
   // 1. Load source code from file
   char* source = ReadText(file);
   glShaderSource( shader, 1, (const char**) &source, NULL );
   free( source );
   // 2. Compile the shader
   fprintf( stderr, "Compile %s\n", file );
   glCompileShader( shader );
   // 3. Check for errors
   PrintShaderLog( shader, file );
   // N. Return ID
   return shader;
}


int CreateShaderProg( char* VertFile, char* FragFile ){
    // Create the shader program, check it for errors, and return the ID
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    // 1. Create program
    int prog = glCreateProgram();
    // 2. Create and compile vertex shader
    int vert = CreateShader( GL_VERTEX_SHADER, VertFile );
    // 3. Create and compile fragment shader
    int frag = CreateShader( GL_FRAGMENT_SHADER, FragFile );
    // 4. Attach vertex shader
    glAttachShader( prog, vert );
    // 5. Attach fragment shader
    glAttachShader( prog, frag );
    // 6. Link program
    glLinkProgram( prog );
    // 7. Check for errors
    PrintProgramLog( prog );
    // N. Return name
    return prog;
}


int CreateShaderGeom( char* VertFile, char* GeomFile, char* FragFile ){
    // Create a complete Vertex --> Geometry --> Fragment shader pipeline
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    // 1. Create program
    int prog = glCreateProgram();
    // 2. Compile shaders
    int vert = CreateShader( GL_VERTEX_SHADER  , VertFile );
    int geom = CreateShader( GL_GEOMETRY_SHADER, GeomFile );
    int frag = CreateShader( GL_FRAGMENT_SHADER, FragFile );
    // 3. Attach shaders
    glAttachShader( prog, vert );
    glAttachShader( prog, geom );
    glAttachShader( prog, frag );
    // 4. Link program
    glLinkProgram( prog );
    // 5. Check for errors
    PrintProgramLog( prog );
    // N. Return name
    return prog;
}
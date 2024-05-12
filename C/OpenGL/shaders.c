////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"



////////// FILE OPERATIONS /////////////////////////////////////////////////////////////////////////

char* ReadText( const char *file ){
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

void PrintShaderLog( int obj, const char* file ){
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

//  Create Shader
//
int CreateShader(int prog,const GLenum type, const char* file){
   //  Create the shader
   int shader = glCreateShader(type);
   //  Load source code from file
   char* source = ReadText(file);
   glShaderSource(shader,1,(const char**)&source,NULL);
   free(source);
   //  Compile the shader
   glCompileShader(shader);
   //  Check for errors
   PrintShaderLog(shader,file);
   //  Attach to shader program
   glAttachShader(prog,shader);
   return shader;
}

//
//  Create Shader Program consisting of only a vertex and fragment shader
//
int CreateShaderProg(const char* vert,const char* frag)
{
   //  Create program
   int prog = glCreateProgram();
   //  Create and compile vertex shader
   if (vert) CreateShader(prog,GL_VERTEX_SHADER,vert);
   //  Create and compile fragment shader
   if (frag) CreateShader(prog,GL_FRAGMENT_SHADER,frag);
   //  Link program
   glLinkProgram(prog);
   //  Check for errors
   PrintProgramLog(prog);
   //  Return name
   return prog;
}


int CreateShaderGeom( const char* VertFile, const char* GeomFile, const char* FragFile ){
    // Create a complete Vertex --> Geometry --> Fragment shader pipeline
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    // 1. Create program
    int prog = glCreateProgram();
    // 2. Compile shaders
    CreateShader( prog, GL_VERTEX_SHADER  , VertFile );
    CreateShader( prog, GL_GEOMETRY_SHADER, GeomFile );
    CreateShader( prog, GL_FRAGMENT_SHADER, FragFile );
    // // 3. Attach shaders
    // glAttachShader( prog, vert );
    // glAttachShader( prog, geom );
    // glAttachShader( prog, frag );
    // 4. Link program
    glLinkProgram( prog );
    // 5. Check for errors
    PrintProgramLog( prog );
    // N. Return name
    return prog;
}



//
int CreateShaderProgCompute( const char* file ){
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    //  Create program
    int prog = glCreateProgram();
    //  Create and compile compute shader
    CreateShader(prog,GL_COMPUTE_SHADER,file);
    //  Link program
    glLinkProgram(prog);
    //  Check for errors
    PrintProgramLog(prog);
    //  Return name
    return prog;
}
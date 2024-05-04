////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"
#include "Matrix4x4f.h"



////////// CONSTANTS & PROGRAM STATE ///////////////////////////////////////////////////////////////

///// Light colors /////
const float Emission[]  = {0.0,0.0,0.0,1.0};
const float Ambient[]   = {0.3,0.3,0.3,1.0};
const float Diffuse[]   = {1.0,1.0,1.0,1.0};
const float Specular[]  = {1.0,1.0,1.0,1.0};
const float Shinyness[] = {16};

///// Transformation matrixes & Vectors /////
float ProjectionMatrix[16];
float ViewMatrix[16];
//  Set lighting parameters using uniforms
float Position[4];

///// Loaded Resources ////////////////////////////////////////////////////
int pi=0; //  Pi texture


///// Vertex Array Object (VAO) Geometry //////////////////////////////////

//  Cube vertex, normal, color and texture data
const float cube_data[] = {
//  X  Y  Z  W   Nx Ny Nz    R G B A   u v
   //  Front
   +1,+1,+1,+1,   0, 0,+1,   1,0,0,1,  1,1,
   -1,+1,+1,+1,   0, 0,+1,   1,0,0,1,  0,1,
   +1,-1,+1,+1,   0, 0,+1,   1,0,0,1,  1,0,
   -1,+1,+1,+1,   0, 0,+1,   1,0,0,1,  0,1,
   +1,-1,+1,+1,   0, 0,+1,   1,0,0,1,  1,0,
   -1,-1,+1,+1,   0, 0,+1,   1,0,0,1,  0,0,
   //  Back                        
   -1,-1,-1,+1,   0, 0,-1,   0,0,1,1,  1,0,
   +1,-1,-1,+1,   0, 0,-1,   0,0,1,1,  0,0,
   -1,+1,-1,+1,   0, 0,-1,   0,0,1,1,  1,1,
   +1,-1,-1,+1,   0, 0,-1,   0,0,1,1,  0,0,
   -1,+1,-1,+1,   0, 0,-1,   0,0,1,1,  1,1,
   +1,+1,-1,+1,   0, 0,-1,   0,0,1,1,  0,1,
   //  Right                       
   +1,+1,+1,+1,  +1, 0, 0,   1,1,0,1,  0,1,
   +1,-1,+1,+1,  +1, 0, 0,   1,1,0,1,  0,0,
   +1,+1,-1,+1,  +1, 0, 0,   1,1,0,1,  1,1,
   +1,-1,+1,+1,  +1, 0, 0,   1,1,0,1,  0,0,
   +1,+1,-1,+1,  +1, 0, 0,   1,1,0,1,  1,1,
   +1,-1,-1,+1,  +1, 0, 0,   1,1,0,1,  1,0,
   //  Left                        
   -1,+1,+1,+1,  -1, 0, 0,   0,1,0,1,  1,1,
   -1,+1,-1,+1,  -1, 0, 0,   0,1,0,1,  0,1,
   -1,-1,+1,+1,  -1, 0, 0,   0,1,0,1,  1,0,
   -1,+1,-1,+1,  -1, 0, 0,   0,1,0,1,  0,1,
   -1,-1,+1,+1,  -1, 0, 0,   0,1,0,1,  1,0,
   -1,-1,-1,+1,  -1, 0, 0,   0,1,0,1,  0,0,
   //  Top                         
   +1,+1,+1,+1,   0,+1, 0,   0,1,1,1,  1,0,
   +1,+1,-1,+1,   0,+1, 0,   0,1,1,1,  1,1,
   -1,+1,+1,+1,   0,+1, 0,   0,1,1,1,  0,0,
   +1,+1,-1,+1,   0,+1, 0,   0,1,1,1,  1,1,
   -1,+1,+1,+1,   0,+1, 0,   0,1,1,1,  0,0,
   -1,+1,-1,+1,   0,+1, 0,   0,1,1,1,  0,1,
   //  Bottom                      
   -1,-1,-1,+1,   0,-1, 0,   1,0,1,1,  0,0,
   +1,-1,-1,+1,   0,-1, 0,   1,0,1,1,  1,0,
   -1,-1,+1,+1,   0,-1, 0,   1,0,1,1,  0,1,
   +1,-1,-1,+1,   0,-1, 0,   1,0,1,1,  1,0,
   -1,-1,+1,+1,   0,-1, 0,   1,0,1,1,  0,1,
   +1,-1,+1,+1,   0,-1, 0,   1,0,1,1,  1,1,
   };

/*
 *  Draw a cube
 */
static void Cube( double x , double y , double z,
                  double dx, double dy, double dz,
                  double th, int shader ){
    // Draw a cube as a VAO
    // Author: Willem A. (Vlakkies) Schreüder  
    static unsigned int cube_vao = 0; // VAO ID on the GPU

    //  Select shader
    glUseProgram( shader );

    //  Once initialized, just bind VAO
    if( cube_vao )
        glBindVertexArray( cube_vao );
    //  Initialize VAO and VBO
    else{
        //  Create cube VAO to bind attribute arrays
        glGenVertexArrays( 1, &cube_vao );
        glBindVertexArray( cube_vao );

        //  Get buffer name
        unsigned int vbo = 0;
        glGenBuffers( 1, &vbo );
        //  Bind VBO
        glBindBuffer( GL_ARRAY_BUFFER, vbo );
        //  Copy cube data to VBO
        glBufferData( GL_ARRAY_BUFFER, sizeof( cube_data ), cube_data, GL_STATIC_DRAW );

        //  Bind arrays
        //  Vertex
        int loc = glGetAttribLocation( shader, "Vertex" );
        glVertexAttribPointer( loc, 4, GL_FLOAT, 0, 52, (void*) 0 );
        glEnableVertexAttribArray( loc );
        //  Normal
        loc = glGetAttribLocation( shader, "Normal" );
        glVertexAttribPointer( loc, 3, GL_FLOAT, 0, 52, (void*) 16 );
        glEnableVertexAttribArray( loc );
        //  Color
        loc = glGetAttribLocation( shader, "Color" );
        glVertexAttribPointer( loc, 4, GL_FLOAT, 0, 52, (void*) 28 );
        glEnableVertexAttribArray( loc );
        //  Texture
        loc = glGetAttribLocation( shader, "Texture" );
        glVertexAttribPointer( loc, 2, GL_FLOAT, 0, 52, (void*) 44 );
        glEnableVertexAttribArray( loc );
    }

    //  Set Projection and View Matrix
    int id = glGetUniformLocation( shader, "ProjectionMatrix" );
    glUniformMatrix4fv(id,1,0,ProjectionMatrix);
    id = glGetUniformLocation(shader,"ViewMatrix");
    glUniformMatrix4fv(id,1,0,ViewMatrix);

    //  Create ModelView matrix
    float ModelViewMatrix[16];
    mat4copy( ModelViewMatrix, ViewMatrix );
    mat4translate( ModelViewMatrix, x, y, z );
    mat4rotate( ModelViewMatrix, th,0,1,0);
    mat4scale( ModelViewMatrix, dx, dy, dz );
    id = glGetUniformLocation( shader, "ModelViewMatrix" );
    glUniformMatrix4fv( id, 1, 0, ModelViewMatrix );
    //  Create Normal matrix
    float NormalMatrix[9];
    mat3normalMatrix( ModelViewMatrix , NormalMatrix );
    id = glGetUniformLocation( shader, "NormalMatrix" );
    glUniformMatrix3fv( id, 1, 0, NormalMatrix );

    //  Set lighting parameters using uniforms
    id = glGetUniformLocation( shader, "Position" );
    glUniform4fv( id, 1, Position );
    id = glGetUniformLocation( shader, "Ambient" );
    glUniform4fv( id, 1, Ambient );
    id = glGetUniformLocation( shader, "Diffuse" );
    glUniform4fv( id, 1, Diffuse );
    id = glGetUniformLocation( shader, "Specular" );
    glUniform4fv(id,1,Specular);

    //  Set material properties as uniforms
    id = glGetUniformLocation( shader, "Ks" );
    glUniform4fv( id, 1, Specular );
    id = glGetUniformLocation( shader, "Ke" );
    glUniform4fv( id, 1, Emission );
    id = glGetUniformLocation( shader, "Shinyness" );
    glUniform1f( id, Shinyness );

    //  Bind Pi texture
    glBindTexture( GL_TEXTURE_2D, pi );
    //  Draw Cube
    glDrawArrays( GL_TRIANGLES, 0, 36 );

    //  Release VAO and VBO
    glBindVertexArray(0);
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
}


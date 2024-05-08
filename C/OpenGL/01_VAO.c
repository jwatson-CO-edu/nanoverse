////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "toolbox.h"
#include "matrix4x4f.h"



////////// CONSTANTS & PROGRAM STATE ///////////////////////////////////////////////////////////////
#ifndef RES
#define RES 1
#endif
int shader[]  = {0,0,0}; //  Shader programs

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


///// Loaded Resources & Program State ////////////////////////////////////
int    pi     =  0; // - Pi texture
int    axes   =  1; // - Display axes
int    move   =  1; // - Move light
int    proj   =  1; // - Projection type
int    th     =  0; // - Azimuth of view angle
int    ph     =  0; // - Elevation of view angle
int    fov    = 55; // - Field of view (for perspective)
int    font   =  0; // - Font texture
double asp    =  1; // - Aspect ratio
double dim    =  3.0; // Size of world
int    zh     = 90; // - Light azimuth
float  Ylight =  2; // - Light elevation


///// Window State ////////////////////////////////////////////////////////

void reshape( int width, int height ){
    // Runs when the user resizes the window
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/

    //  Ratio of the width to the height of the window
    asp = (height > 0) ? ((double) width / height) : 1;
    //  Set the viewport to the entire window
    glViewport( 0, 0, RES*width, RES*height );
    //  Set projection
    Project( proj ? fov : 0, asp, dim );
}

////////// CUBE VAO EXAMPLE ////////////////////////////////////////////////////////////////////////

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
static unsigned int cube_vao = 0; // VAO ID on the GPU
static void Cube(double x,double y,double z,
                 double dx,double dy,double dz,
                 double th)
{
   //  Select shader
   glUseProgram(shader[0]);

   //  Once initialized, just bind VAO
   if (cube_vao)
      glBindVertexArray(cube_vao);
   //  Initialize VAO and VBO
   else
   {
      //  Create cube VAO to bind attribute arrays
      glGenVertexArrays(1,&cube_vao);
      glBindVertexArray(cube_vao);

      //  Get buffer name
      unsigned int vbo=0;
      glGenBuffers(1,&vbo);
      //  Bind VBO
      glBindBuffer(GL_ARRAY_BUFFER,vbo);
      //  Copy cube data to VBO
      glBufferData(GL_ARRAY_BUFFER,sizeof(cube_data),cube_data,GL_STATIC_DRAW);

      //  Bind arrays
      //  Vertex
      int loc = glGetAttribLocation(shader[0],"Vertex");
      glVertexAttribPointer(loc,4,GL_FLOAT,0,52,(void*) 0);
      glEnableVertexAttribArray(loc);
      //  Normal
      loc = glGetAttribLocation(shader[0],"Normal");
      glVertexAttribPointer(loc,3,GL_FLOAT,0,52,(void*)16);
      glEnableVertexAttribArray(loc);
      //  Color
      loc  = glGetAttribLocation(shader[0],"Color");
      glVertexAttribPointer(loc,4,GL_FLOAT,0,52,(void*)28);
      glEnableVertexAttribArray(loc);
      //  Texture
      loc  = glGetAttribLocation(shader[0],"Texture");
      glVertexAttribPointer(loc,2,GL_FLOAT,0,52,(void*)44);
      glEnableVertexAttribArray(loc);
   }

   //  Set Projection and View Matrix
   int id = glGetUniformLocation(shader[0],"ProjectionMatrix");
   glUniformMatrix4fv(id,1,0,ProjectionMatrix);
   id = glGetUniformLocation(shader[0],"ViewMatrix");
   glUniformMatrix4fv(id,1,0,ViewMatrix);

   //  Create ModelView matrix
   float ModelViewMatrix[16];
   mat4copy(ModelViewMatrix , ViewMatrix);
   mat4translate(ModelViewMatrix , x,y,z);
   mat4rotate(ModelViewMatrix , th,0,1,0);
   mat4scale(ModelViewMatrix , dx,dy,dz);
   id = glGetUniformLocation(shader[0],"ModelViewMatrix");
   glUniformMatrix4fv(id,1,0,ModelViewMatrix);
   //  Create Normal matrix
   float NormalMatrix[9];
   mat3normalMatrix(ModelViewMatrix , NormalMatrix);
   id = glGetUniformLocation(shader[0],"NormalMatrix");
   glUniformMatrix3fv(id,1,0,NormalMatrix);

   //  Set lighting parameters using uniforms
   id = glGetUniformLocation(shader[0],"Position");
   glUniform4fv(id,1,Position);
   id = glGetUniformLocation(shader[0],"Ambient");
   glUniform4fv(id,1,Ambient);
   id = glGetUniformLocation(shader[0],"Diffuse");
   glUniform4fv(id,1,Diffuse);
   id = glGetUniformLocation(shader[0],"Specular");
   glUniform4fv(id,1,Specular);

   //  Set material properties as uniforms

   const float White[] = {1,1,1,1};
   id = glGetUniformLocation(shader[1],"Kd");
   glUniform4fv(id,1,White);
   id = glGetUniformLocation(shader[2],"Kd");
   glUniform4fv(id,1,White);

   id = glGetUniformLocation(shader[0],"Ks");
   glUniform4fv(id,1,Specular);
   id = glGetUniformLocation(shader[0],"Ke");
   glUniform4fv(id,1,Emission);
   id = glGetUniformLocation(shader[0],"Shinyness");
   glUniform1f(id,Shinyness[0]);

   //  Bind Pi texture
   glBindTexture(GL_TEXTURE_2D,pi);
   //  Draw Cube
   glDrawArrays(GL_TRIANGLES,0,36);

   //  Release VAO and VBO
   glBindVertexArray(0);
   glBindBuffer(GL_ARRAY_BUFFER,0);
}



////////// RENDERING ///////////////////////////////////////////////////////////////////////////////


void display(){
    // Draw the frame
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/

    // Clear the image
	
	

    // 1. Erase the window and the depth buffer
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );

    // 2. Enable Z-buffering in OpenGL
    // glEnable( GL_DEPTH_TEST );

    // 3. Create Projection matrix
    mat4identity(ProjectionMatrix);
    if( proj )
        mat4perspective( ProjectionMatrix , fov, asp, dim/16, 16*dim );
    else
        mat4ortho( ProjectionMatrix , -dim*asp, +dim*asp, -dim, +dim, -dim, +dim );
    //  Create View matrix
    mat4identity( ViewMatrix );
    if( proj ){
        double Ex = -2*dim*Sin(th)*Cos(ph);
        double Ey = +2*dim        *Sin(ph);
        double Ez = +2*dim*Cos(th)*Cos(ph);
        mat4lookAt( ViewMatrix , Ex, Ey, Ez , 0,0,0 , 0, Cos( ph ), 0 );
    }else{
        mat4rotate( ViewMatrix, ph, 1, 0, 0 );
        mat4rotate( ViewMatrix, th, 0, 1, 0 );
    }
    //  Light position
    Position[0] = 4.0*Cos(zh);
    Position[1] = Ylight;
    Position[2] = 4.0*Sin(zh);
    Position[3] = 1.0f;

    //  Now draw the scene (just a cube for now)
    //  To do other objects create a VBO and VAO for each object
    Cube( 0,0,0, 
          1,1,1, 
          0 );

    //  Draw axes
    //    if (axes) Axes();

    //  Revert to fixed pipeline for labels
    glUseProgram(0);

    //  Display parameters
    glWindowPos2i( 5, 5 );
    Print( "Angle=%d,%d  Dim=%.1f Projection=%s", th, ph, dim, proj ? "Perpective":"Orthogonal" );
    //  Render the scene and make it visible
    ErrCheck("display");
    glFlush();
    glutSwapBuffers();
}



////////// ANIMATION ///////////////////////////////////////////////////////////////////////////////

void idle(){
   //  Elapsed time in seconds
   double t = glutGet( GLUT_ELAPSED_TIME ) / 1000.0;
   if( move )  zh = fmod(90*t,360.0);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}



////////// USER INTERACTION ////////////////////////////////////////////////////////////////////////

void special(int key,int x,int y){
    // Handle arrow and other special keys
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    //  Right arrow key - increase angle by 5 degrees
    if (key == GLUT_KEY_RIGHT)
        th += 5;
    //  Left arrow key - decrease angle by 5 degrees
    else if (key == GLUT_KEY_LEFT)
        th -= 5;
    //  Up arrow key - increase elevation by 5 degrees
    else if (key == GLUT_KEY_UP)
        ph += 5;
    //  Down arrow key - decrease elevation by 5 degrees
    else if (key == GLUT_KEY_DOWN)
        ph -= 5;
    //  PageUp key - increase dim
    else if (key == GLUT_KEY_PAGE_DOWN)
        dim += 0.1;
    //  PageDown key - decrease dim
    else if (key == GLUT_KEY_PAGE_UP && dim>1)
        dim -= 0.1;
    //  Keep angles to +/-360 degrees
    th %= 360;
    ph %= 360;
    //  Update projection
    Project(proj?fov:0,asp,dim);
    //  Tell GLUT it is necessary to redisplay the scene
    glutPostRedisplay();
}


void key( unsigned char ch, int x, int y ){
    // Handle main alphanumeric keys
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    //  Exit on ESC
    if (ch == 27)
        exit(0);
    //  Reset view angle
    else if (ch == '0')
        th = ph = 0;
    //  Toggle axes
    else if (ch == 'a' || ch == 'A')
        axes = 1-axes;
    //  Toggle projection type
    else if (ch == 'p' || ch == 'P')
        proj = 1-proj;
    //  Toggle light movement
    else if (ch == 's' || ch == 'S')
        move = 1-move;
    //  Light elevation
    else if (ch == '+')
        Ylight += 0.1;
    else if (ch == '-')
        Ylight -= 0.1;
    //  Reproject
    Project(proj?fov:0,asp,dim);
    //  Tell GLUT it is necessary to redisplay the scene
    glutPostRedisplay();
}


int main(int argc,char* argv[])
{
   //  Initialize GLUT
   glutInit(&argc,argv);
   //  Request double buffered, true color window with Z buffering at 600x600
   glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
   glutInitWindowSize(600,600);
   glutCreateWindow("Shaders - OpenGL4");

   //  Set callbacks
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutSpecialFunc(special);
   glutKeyboardFunc(key);
   glutIdleFunc(idle);

    // Enable z-testing at the full ranger
	glEnable( GL_DEPTH_TEST );
	glDepthRange( 0.0f , 1.0f );
	// glEnable( GL_CULL_FACE );  
    

   //  Load textures
   pi   = LoadTexBMP("resources/pi.bmp");
//    font = LoadTexBMP("resources/font.bmp");
//    //  Switch font to nearest
//    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
   //  Create Shader Programs
   shader[0] = CreateShaderProg("shaders/gl4pix.vert","shaders/gl4pix.frag");
   shader[1] = CreateShaderProg("shaders/gl4fix.vert","shaders/gl4fix.frag");
   shader[2] = CreateShaderGeom("shaders/gl4tex.vert","shaders/gl4tex.geom","shaders/gl4tex.frag");
   //  Pass control to GLUT so it can interact with the user
   ErrCheck("init");
   glutMainLoop();
   return 0;
}
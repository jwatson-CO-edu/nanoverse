////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include <cstdarg>

#include "include/toolbox.hpp"



////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x ){  return cos( (x) * 3.1415927 / 180 );  }
double Sin( double x ){  return sin( (x) * 3.1415927 / 180 );  }
double Tan( double x ){  return tan( (x) * 3.1415927 / 180 );  }
float  Cosf( float x ){  return cosf( (x) * 3.1415927f / 180.0f );  }
float  Sinf( float x ){  return sinf( (x) * 3.1415927f / 180.0f );  }
float  Tanf( float x ){  return tanf( (x) * 3.1415927f / 180.0f );  }
float  Atan2f( float y, float x ){  return (atan2f( y, x ) * 3.1415927f / 180.0f);  }



////////// LINEAR ALGEBRA //////////////////////////////////////////////////////////////////////////
mat4f identity_mtx44f( void ){  return mat4f( 1.0f );  }



////////// POLYHEDRA ///////////////////////////////////////////////////////////////////////////////

void Vertex( int th, int ph ){
    // Draw vertex in polar coordinates
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    double x = Cos(th)*Cos(ph);
    double y = Sin(th)*Cos(ph);
    double z =         Sin(ph);
    glNormal3d(x,y,z);
    glTexCoord2d(th/360.0,ph/180.0+0.5);
    glVertex3d(x,y,z);
}



////////// OPENGL SYSTEM ///////////////////////////////////////////////////////////////////////////


void ErrCheck( const char* where ){
   // Check for OpenGL errors and print to stderr
   // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
   int err = glGetError();
   if(err) cerr << "ERROR: " << gluErrorString(err) << ", " << where << "\n";
}


void Fatal( const char* format , ... ){
   // Print message to stderr and exit
   // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
   va_list args;
   va_start( args, format );
   vfprintf( stderr, format, args );
   va_end( args );
   exit(1);
}


void Print(const char* format , ...){
    // Print raster letters to the viewport
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    char    buf[LEN];
    char*   ch = buf;
    va_list args;
    // 1. Turn the parameters into a character string
    va_start( args, format );
    vsnprintf( buf, LEN, format, args );
    va_end( args );
    // 2. Display the characters one at a time at the current raster position
    while( *ch ){  glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *ch++ );  }
}


void Project( double fov, double asp, double dim ){
    // Set projection
    // Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/
    // 1. Tell OpenGL we want to manipulate the projection matrix
    glMatrixMode( GL_PROJECTION );
    // 2. Undo previous transformations
    glLoadIdentity();
    // 3. Perspective transformation
    if( fov )
        gluPerspective( fov, asp, dim/16, 16*dim );
    // 4. Orthogonal transformation
    else
        glOrtho( -asp*dim, asp*dim, -dim, +dim, -dim, +dim );
    //  Switch to manipulating the model matrix
    glMatrixMode( GL_MODELVIEW );
    //  Undo previous transformations
    glLoadIdentity();
}



////////// OPENGL POINTS //////////////////////////////////////////////////////////////////////////

void glVtx4f( const vec4f v ){  glVertex4f( v.x , v.y , v.z, v.w );  } // Set vertex with a vector
void glNrm4f( const vec4f n ){  glNormal3f( n.x , n.y , n.z      );  } // Set normal with a vector
void glClr4f( const vec4f c ){  glColor4f(  c.r , c.g , c.b, c.a );  } // Set color with a vector



////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

void look( const Camera3D camera ){
    // Set camera position, target, and orientation
    gluLookAt( (double) camera.eyeLoc[0], (double) camera.eyeLoc[1], (double) camera.eyeLoc[2],  
               (double) camera.lookPt[0], (double) camera.lookPt[1], (double) camera.lookPt[2],  
               (double) camera.upVctr[0], (double) camera.upVctr[1], (double) camera.upVctr[2] );
}
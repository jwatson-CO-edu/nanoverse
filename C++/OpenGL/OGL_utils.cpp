////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include <cstdarg>

#include "include/toolbox.hpp"

////////// GLOBAL SETTINGS /////////////////////////////////////////////////////////////////////////
float _DRAW_DIST_MIN = 50.0f / 16.0f; // Scale Dimension
float _DRAW_DIST_MAX = 50.0f * 16.0f; // Scale Dimension
int   _FOV_DEG /*-*/ =   55; // - Field of view (for perspective)
float _TARGET_FPS    =   60.0f; // Desired framerate
int   _WINDOW_W /**/ = 1200;
int   _WINDOW_H /**/ =  900;
float _ASPECT_RAT    =    0.0f; // Aspect ratio

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
   // Check for OpenGL errors and print to `stderr`
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


void Print( const char* format , ... ){
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


void project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	// 1. Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	// Undo previous transformations
	glLoadIdentity();
	gluPerspective( _FOV_DEG , // ------ Field of view angle, in degrees, in the y direction.
					_ASPECT_RAT , // ----------- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					_DRAW_DIST_MIN , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					_DRAW_DIST_MAX ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	// 2. Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// 3. Undo previous transformations
	glLoadIdentity();
}


void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
    // Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// 1. Calc the aspect ratio: width to the height of the window
	_ASPECT_RAT = ( height > 0 ) ? (float) width / height : 1;
	// 2. Set the viewport to the entire window
	glViewport( 0, 0, width, height );
	// 3. Set projection
	project();
}


void clear_screen(){
    // Erase the window and the depth buffer
    glClearDepth( 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glLoadIdentity();
}

void set_near_far_draw_distance( float nearDist, float farDist ){
    // Set distances for clipping planes
    _DRAW_DIST_MIN = nearDist;
    _DRAW_DIST_MAX = farDist;
}



////////// OPENGL POINTS //////////////////////////////////////////////////////////////////////////

void glVtx4f( const vec4f v ){  glVertex4f( v.x , v.y , v.z, v.w );  } // Set vertex with a vector
void glNrm4f( const vec4f n ){  glNormal3f( n.x , n.y , n.z      );  } // Set normal with a vector
void glClr4f( const vec4f c ){  glColor4f(  c.r , c.g , c.b, c.a );  } // Set color with a vector



////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

Camera3D::Camera3D( const vec3f& eyePsn, const vec3f& lukPnt, const vec3f& upDrct ){
    eyeLoc = eyePsn;
    lookPt = lukPnt;
    upVctr = upDrct;    
}


void Camera3D::look(){
    // Set camera position, target, and orientation
    gluLookAt( (double) eyeLoc[0], (double) eyeLoc[1], (double) eyeLoc[2],  
               (double) lookPt[0], (double) lookPt[1], (double) lookPt[2],  
               (double) upVctr[0], (double) upVctr[1], (double) upVctr[2] );
}


void Camera3D::move_to( const vec3f& eyePsn ){  eyeLoc = eyePsn;  }
void Camera3D::look_at( const vec3f& lukPnt ){  lookPt = lukPnt;  }
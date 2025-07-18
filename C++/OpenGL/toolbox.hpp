#ifndef TOOLBOX_HPP // This pattern is to prevent symbols to be loaded multiple times
#define TOOLBOX_HPP // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Defines /////////////////////////////////////////////////////////////
// NOTE: It's just a good idea to put `#define`s before `#include`s because they might trigger important macros
#define _USE_MATH_DEFINES 
#define GL_GLEXT_PROTOTYPES // REQUIRED HERE: Get all GL prototypes // WARNING: MUST appear BEFORE ALL GL includes!
#define LEN 8192 // ---------- Maximum length of text string

///// Imports /////////////////////////////////////////////////////////////
/// Standard ////
#include <iostream>
using std::cout, std::cerr, std::endl, std::flush;
#include <memory>
using std::shared_ptr;
#include <limits>
#include <vector>
using std::vector;


/// OpenGL + GLUT ////
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

#include <glm/glm.hpp>
using glm::vec2;
using glm::vec3;
using glm::mat4;
#include <glm/vec4.hpp> 
using glm::vec4;
#include <glm/gtc/matrix_transform.hpp>
using glm::translate, glm::scale, glm::rotate;
#include <glm/gtc/type_ptr.hpp>
using glm::value_ptr;


////////// TYPE DEFINES ////////////////////////////////////////////////////////////////////////////

typedef unsigned char ubyte;
typedef unsigned int  uint;
typedef unsigned long ulong;
typedef vec2 /*----*/ vec2f;
typedef vec3 /*----*/ vec3f;
typedef vec4 /*----*/ vec4f;
typedef mat4 /*----*/ mat4f;


////////// VECTOR STRUCTS //////////////////////////////////////////////////////////////////////////

typedef struct{
    union{ uint v0; uint f0; };
    union{ uint v1; uint f1; };
    union{ uint v2; uint f2; };
} vec3u;


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// OGL_utils.cpp //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// TRIGONOMETRY ////////////////////////////////////////////////////////////////////////////

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
double Cos( double x );
double Sin( double x );
double Tan( double x );
float  Cosf( float x );
float  Sinf( float x );
float  Tanf( float x );
float  Atan2f( float y, float x );



////////// LINEAR ALGEBRA //////////////////////////////////////////////////////////////////////////
mat4f identity_mtx44f( void );


////////// POLYHEDRA ///////////////////////////////////////////////////////////////////////////////
static void Vertex( int th, int ph );


////////// OPENGL SYSTEM ///////////////////////////////////////////////////////////////////////////
void ErrCheck( const char* where );
void Fatal( const char* format , ... );
void Print(const char* format , ...);
void Project( double fov, double asp, double dim );



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// load_assets.c //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Author: Willem A. (Vlakkies) Schreüder, https://www.prinmath.com/

unsigned int LoadTexBMP( const char* file );
int  LoadOBJ( const char* file );



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// LightSource.cpp ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct{
    // Light source for default Phong shading
    uint  ID; // ----- GL light source enum
    vec4f position; // Position in the world frame
    vec4f ambient;
	vec4f diffuse;
	vec4f specular;
}LightSource;

LightSource* make_white_light_source( const vec4f posn, uint sourcEnum, int ambientPrcnt, int diffusePrcnt, int specularPrcnt );
void /*---*/ illuminate_with_source( LightSource* lite );


#endif
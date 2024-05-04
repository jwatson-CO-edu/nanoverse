#ifndef VECTOR_F_OPS_H // This pattern is to prevent symbols to be loaded multiple times
#define VECTOR_F_OPS_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include <math.h>
#include "OGL_base.h"



////////// VECTOR STRUCTS //////////////////////////////////////////////////////////////////////////

typedef struct{
    union{ float x; float r; };
    union{ float y; float g; };
    union{ float z; float b; };
    union{ float w; float a; };
} vec4f;

typedef struct{
  float x;
  float y;
} vec2f;

typedef struct{
    union{ uint v0; uint f0; };
    union{ uint v1; uint f1; };
    union{ uint v2; uint f2; };
} vec3u;

////////// SCALED 3D VECTORS ///////////////////////////////////////////////////////////////////////

vec4f make_vec4f( float x, float y, float z ); // Make a 3D float vector with scale = 1.0 from three floats
vec4f make_0_vec4f( void ); // ------------------ Make a 3D zero float vector with scale = 1.0

vec4f sub_vec4f( const vec4f u, const vec4f v ); // - Calc `u` - `v` = `r`, R^3
vec4f add_vec4f( const vec4f u, const vec4f v ); // - Calc `u` + `v` = `r`, R^3
float dot_vec4f( const vec4f u, const vec4f v ); // - Calc `u` * `v` = `r`, R^3
float norm_vec4f( const vec4f vec ); // ------------- Euclidean length of an R^3
float diff_vec4f( const vec4f u, const vec4f v ); //- Euclidean length of `u`-`v`
vec4f unit_vec4f( const vec4f vec ); // ------------- Calc the unit direction of `vec` and return it, R^3
vec4f cross_vec4f( const vec4f u, const vec4f v ); // Calc `u` X `v` = `p`, R^3
vec4f div_vec4f( const vec4f u, float d ); // ------- Calc `u` / `f` = `r`, R^3
vec4f scale_vec4f( const vec4f u, float f ); // ----- Calc `u` * `f` = `r`, R^3


///// 3D Segments & Triangles /////////////////////////////////////////////

vec4f seg_center( const vec4f v0, const vec4f v1 ); // --------------- Calc centroid of 2 R^3 points
vec4f tri_center( const vec4f v0, const vec4f v1, const vec4f v2 ); // Calc centroid of 3 R^3 points
// Find the normal vector `n` of a triangle defined by CCW vertices in R^3: {`v0`,`v1`,`v2`}
vec4f get_CCW_tri_norm( const vec4f v0, const vec4f v1, const vec4f v2 ); 


////////// 2D VECTORS //////////////////////////////////////////////////////////////////////////////

vec2f make_vec2f( float x, float y ); // Create a 2D float vector from 2 floats


////////// 2D <---> 3D /////////////////////////////////////////////////////////////////////////////

// Project the local 2D point to the global 3D frame
vec4f lift_pnt_2D_to_3D( const vec2f pnt2f, const vec4f origin, const vec4f xBasis, const vec4f yBasis ); 
// Project the local 2D vector to the global 3D frame
vec4f lift_vec_2D_to_3D( const vec2f vct2f, const vec4f xBasis, const vec4f yBasis );


////////// UINT VECTORS ////////////////////////////////////////////////////////////////////////////

vec3u make_vec3u( uint u0, uint u1, uint u2 ); // Create a 3D uint vector


////////// OPENGL HELPERS //////////////////////////////////////////////////////////////////////////

void glVtx4f( const vec4f v ); // Set vertex with a vector
void glNrm4f( const vec4f n ); // Set normal with a vector
void glClr4f( const vec4f c ); // Set color with a vector


#endif
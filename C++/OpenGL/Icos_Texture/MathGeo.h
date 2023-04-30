#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
MathGeo_ASP.h
James Watson , 2018 October
Basic Math and 2D / 3D Geometry Utilities
NOTE: This library can be templated on either float/double using the 'MG_FLOAT'/'MG_DUBBL' flags

Template Version: 2018-07-16
***********/

#ifndef MATH_GEO_H // This pattern is to prevent symbols to be loaded multiple times
#define MATH_GEO_H // from multiple imports

#define MG_FLOAT // Use floats for all Eigen operations
//~ #define MG_DUBBL // Use doubles for all Eigen operations

// ~~ Includes ~~
// ~ Eigen ~
#include <Eigen/Core> // ---- The living heart of Eigen
#include <Eigen/Dense> // --- Cross Product , etc.
#include <Eigen/Geometry> //- Quaternion , etc
// ~ Local ~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/

// ~~ Shortcuts and Aliases ~~
// ~ Eigen ~
#ifdef MG_FLOAT
	using vec2e = Eigen::Vector2f;
	using vec3e = Eigen::Vector3f;
	using matXe = Eigen::MatrixXf;
	using typeF = float;
	#define random rand_float
#endif
#ifdef MG_DUBBL
	using vec2e = Eigen::Vector2d;
	using vec3e = Eigen::Vector3d;
	using matXe = Eigen::MatrixXd;
	using typeF = double;
	#define random rand_dbbl
#endif
using vec2i = Eigen::Vector2i;
using matXi = Eigen::MatrixXi;



// ~~ Constants ~~
const size_t ICOS_SPHERE_DIVISN  =   7; // ---- Divide each icosahedron triangle into 28 triangles
const size_t CIRCLE_DIVISION     = 180; // ---- Divide each circle into 300 Segments
const typeF  GEO_CRIT_ANG /*- */ =   0.0349; // Angle Criterion , 2deg

// === Classes and Structs =================================================================================================================

// == class Icosahedron_e ==

// Geometry based on Paul Bourke's excellent article:
//   Platonic Solids (Regular polytopes in 3D)
//   http://astronomy.swin.edu.au/~pbourke/polyhedra/platonic/

class Icosahedron_e{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// ~ Constants ~
	typeF sqrt5 = (typeF) sqrt( 5.0d ); // ----------------------------------- Square root of 5
	typeF phi   = (typeF)( 1.0d + sqrt5 ) * 0.5d; // ------------------------- The Golden Ratio
	typeF ratio = (typeF)sqrt( 10.0d + ( 2.0d * sqrt5 ) ) / ( 4.0d * phi ); // ratio of edge length to radius
	
	// ~ Variables ~
	vec3e center;
	typeF /* -- */ radius;
	typeF /* -- */ a; 
	typeF /* -- */ b; 
	matXe V; // Points of the mesh
	matXi F; // Facets corresponding to the points V
	matXe N; // Mesh normals
	
	// ~ Constructors & Destructors ~
	Icosahedron_e(); // ------------------------------ Default constructor
	Icosahedron_e( typeF rad , const vec3e& cntr ); // Parameter constructor
	~Icosahedron_e(); // ----------------------------- Destructor
	
	// ~ Getters ~
	matXe& get_vertices();
	matXi& get_facets();
	matXe& get_normals();
	
protected:
	// ~ Init ~
	void _init( typeF rad , const vec3e& cntr );
};

// __ End Icosahedron_e __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

// == CSCI 5229 ==

// Cosine and Sine in degrees
typeF Cos( typeF x );
typeF Sin( typeF x );

vec2e polr_2_cart_0Y( const vec2e& polarCoords ); // Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ 

// Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified 
matXe circ_space( float dia , uint numPts , const vec2e& center );

matXe pts_XY_at_Z( const matXe& XY , typeF Z ); // Set 'XY' points in 3D space at 'Z' height

vec3e sphr_2_cart_pnt( typeF r , typeF th , typeF ph ); // For the given spherical coordinates , Return the Cartesian point

vec3e vec_sphr( typeF r , typeF th , typeF ps );

matXi equilateral_tri_pixels( const vec2e& center , typeF radius );

// __ End 8229 __


// == Trigonometry ==

typeF degrees( typeF angRad );
typeF radians( typeF angDeg );

// __ End Trig __


// == Geo 2D ==

matXe equilateral_tri_vertices( const vec2e& center , typeF radius );

vec2e vec2e_random();

vec2e rand_corners( const vec2e& corner1 , const vec2e& corner2 );

vec2e sample_from_box( const matXe& box );

// __ End 2D __


// == Geo 3D ==

typeF angle_between( const vec3e& vec1 , const vec3e& vec2 );

vec3e vec3e_random();

vec3e vec3e_rand_corners( const vec3e& corner1 , const vec3e& corner2 );

matXe sample_from_AABB( size_t N , const matXe& aabb );
vec3e sample_from_AABB( const matXe& aabb );

vec3e get_any_perpendicular( const vec3e& query , typeF CRIT_ANG = GEO_CRIT_ANG );

// __ End 3D __


// == Mesh Operations ==

vec3e get_CCW_tri_norm( const vec3e& v0 , const vec3e& v1 , const vec3e& v2 );

vec3e get_CCW_tri_norm( const matXe& V );

matXe N_from_VF( const matXe& V , const matXi& F );

// __ End Mesh __

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/


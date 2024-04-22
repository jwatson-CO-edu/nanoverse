/***********  
MathGeo_ASP.cpp
James Watson , 2018 October
Basic Math and 2D / 3D Geometry Utilities
NOTE: This library can be templated on either float/double using the 'MG_FLOAT'/'MG_DUBBL' flags

Template Version: 2018-06-07
***********/

#include "MathGeo.h"

// === Classes and Structs =================================================================================================================

// == class Icosahedron_d ==

// Geometry based on Paul Bourke's excellent article:
//   Platonic Solids (Regular polytopes in 3D)
//   http://paulbourke.net/geometry/platonic/

// ~ Constructors & Destructors ~
void Icosahedron_e::_init( typeF rad , const vec3e& cntr ){
	center = cntr;
	radius = rad;
	a = ( radius / ratio ) * 0.5;
	b = ( radius / ratio ) / ( 2.0f * phi );
	V = matXe::Zero( 12 ,  3 ); // Points of the mesh
	F = matXi::Zero( 20 ,  3 ); // Facets corresponding to the points V
	
	// Define the icosahedron's 12 vertices:
	V.row(  0 ) = cntr + vec3e(  0 ,  b , -a );
	V.row(  1 ) = cntr + vec3e(  b ,  a ,  0 );
	V.row(  2 ) = cntr + vec3e( -b ,  a ,  0 );
	V.row(  3 ) = cntr + vec3e(  0 ,  b ,  a );
	V.row(  4 ) = cntr + vec3e(  0 , -b ,  a );
	V.row(  5 ) = cntr + vec3e( -a ,  0 ,  b );
	V.row(  6 ) = cntr + vec3e(  0 , -b , -a );
	V.row(  7 ) = cntr + vec3e(  a ,  0 , -b );
	V.row(  8 ) = cntr + vec3e(  a ,  0 ,  b );
	V.row(  9 ) = cntr + vec3e( -a ,  0 , -b );
	V.row( 10 ) = cntr + vec3e(  b , -a ,  0 );
	V.row( 11 ) = cntr + vec3e( -b , -a ,  0 );
	
	// Define the icosahedron's 20 triangular faces:
	//   CCW            ||  CW
    F <<  2 ,  1 ,  0 , //~  0 ,  1 ,  2 ,
          1 ,  2 ,  3 , //~  3 ,  2 ,  1 ,
          5 ,  4 ,  3 , //~  3 ,  4 ,  5 ,
          4 ,  8 ,  3 , //~  3 ,  8 ,  4 ,
          7 ,  6 ,  0 , //~  0 ,  6 ,  7 ,
          6 ,  9 ,  0 , //~  0 ,  9 ,  6 ,
         11 , 10 ,  4 , //~  4 , 10 , 11 ,
         10 , 11 ,  6 , //~  6 , 11 , 10 ,
          9 ,  5 ,  2 , //~  2 ,  5 ,  9 ,
          5 ,  9 , 11 , //~ 11 ,  9 ,  5 ,
          8 ,  7 ,  1 , //~  1 ,  7 ,  8 ,
          7 ,  8 , 10 , //~ 10 ,  8 ,  7 ,
          2 ,  5 ,  3 , //~  3 ,  5 ,  2 ,
          8 ,  1 ,  3 , //~  3 ,  1 ,  8 ,
          9 ,  2 ,  0 , //~  0 ,  2 ,  9 ,
          1 ,  7 ,  0 , //~  0 ,  7 ,  1 ,
         11 ,  9 ,  6 , //~  6 ,  9 , 11 ,
          7 , 10 ,  6 , //~  6 , 10 ,  7 ,
          5 , 11 ,  4 , //~  4 , 11 ,  5 ,
         10 ,  8 ,  4 ; //~  4 ,  8 , 10 ;
         
	 // Define the normals
	 N = N_from_VF( V , F );
}

Icosahedron_e::Icosahedron_e(){ _init( 1.0d , vec3e( 0.0d , 0.0d , 0.0d ) ); } // Default constructor

Icosahedron_e::Icosahedron_e( typeF rad , const vec3e& cntr ){ _init( rad , cntr ); } // Parameter constructor

Icosahedron_e::~Icosahedron_e(){ /* Nothing to do here! */ } // Destructor

// ~ Getters ~
matXe& Icosahedron_e::get_vertices(){ return V; }
matXi& Icosahedron_e::get_facets(){   return F; };
matXe& Icosahedron_e::get_normals(){  return N; };

// __ End Icosahedron_d __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

// == CSCI 5229 ==

// Cosine and Sine in degrees
// Author: Willem A. (Vlakkies) Schreüder  
typeF Cos( typeF x ){ return (typeF)cos( (x) * 3.1415927 / 180 ); }
typeF Sin( typeF x ){ return (typeF)sin( (x) * 3.1415927 / 180 ); }

vec2e polr_2_cart_0Y( const vec2e& polarCoords ){ // 0 angle is +Y North 
    // Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ 
    return vec2e{ polarCoords[0] * sinf( polarCoords[1] ) , polarCoords[0] * cosf( polarCoords[1] ) };
}

matXe circ_space( typeF dia , uint numPts , const vec2e& center ){
    // Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified 
    typeF div = 2.0 * M_PI / numPts;
    matXe circPts = matXe::Zero( numPts , 2 );
    vec2e offset;
    for( uint pntDex = 0 ; pntDex < numPts ; pntDex++ ){
		offset = polr_2_cart_0Y( vec2e{ dia/2 , pntDex * div } );
		offset = center + offset;
		circPts.row( pntDex ) = offset;
	}
    return circPts;
}

matXe pts_XY_at_Z( const matXe& XY , typeF Z ){
	// Set 'XY' points in 3D space at 'Z' height
	uint len = XY.rows();
	matXe rtnPts = matXe::Zero( len , 3 );
	for( uint i = 0 ; i < len ; i++ ){  rtnPts.row(i) = vec3e{ XY(i,0) , XY(i,1) , Z };  }
	return rtnPts;
}

vec3e sphr_2_cart_pnt( typeF r , typeF th , typeF ph ){
	// For the given spherical coordinates , Return the Cartesian point
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	return vec3e{
		r * Sin( th ) * Cos( ph ) , 
		r * Sin( ph ) , 
		r * Cos( th ) * Cos( ph )
	};
}

vec3e vec_sphr( typeF r , typeF th , typeF ps ){
	// Return a vertex in spherical coordinates , Theta axis is Z+
	// Based on code provided by Willem A. (Vlakkies) Schreüder  
	return vec3e{ r * Cos( th ) * Cos( ps ) , 
				  r * Sin( th ) * Cos( ps ) , 
				  r * Sin( ps ) };
}

// __ End 8229 __


// == Trigonometry ==

typeF degrees( typeF angRad ){  return (typeF)angRad * 180.0 / M_PI;  }
typeF radians( typeF angDeg ){  return (typeF)angDeg * M_PI / 180.0;  }

// __ End Trig __


// == Geo 3D ==

typeF angle_between( const vec3e& vec1 , const vec3e& vec2 ){
	// Get the angle between two R3 vectors , radians
	typeF angle = acos( vec1.normalized().dot( vec2.normalized() ) ); // for now assume that there are no special cases
	if( isnan( angle ) ){
		if( vec1.normalized() == vec2.normalized() ){ return (typeF)0.0; }
		else { return (typeF)M_PI; }
	} else { return (typeF)angle; }
}

vec3e vec3e_random(){  return vec3e( random() , random() , random() );  }

vec3e vec3e_rand_corners( const vec3e& corner1 , const vec3e& corner2 ){
	vec3e span = corner2 - corner1;
	vec3e sample = vec3e_random();
	return vec3e{ corner1(0)+span(0)*sample(0) , corner1(1)+span(1)*sample(1) , corner1(2)+span(2)*sample(2) };
}

matXe sample_from_AABB( size_t N , const matXe& aabb ){
	// Return 'N' uniform, random samples from AABB
	matXe rtnMatx = matXe::Zero( N , 3 );
	vec3e crnr1 = aabb.row(0);
	vec3e crnr2 = aabb.row(1);
	for( size_t i = 0 ; i < N ; i++ ){
		rtnMatx.row(i) = vec3e_rand_corners( crnr1 , crnr2 );
	}
	return rtnMatx;
}

vec3e sample_from_AABB( const matXe& aabb ){
	// Return a uniform, random samples from AABB
	vec3e crnr1 = aabb.row(0);
	vec3e crnr2 = aabb.row(1);
	return vec3e_rand_corners( crnr1 , crnr2 );
} 

vec3e get_any_perpendicular( const vec3e& query , typeF CRIT_ANG ){
	// Get any unit vector that is perpendicular to 'query'
	vec3e op = vec3e_random();
	while(  eq( angle_between( op , query ) , (typeF)0.0 , CRIT_ANG )  ){  op = vec3e_random();  }
	return op.cross( query ).normalized();
}

// __ End 3D __


// == Mesh Operations ==

vec3e get_CCW_tri_norm( const vec3e& v0 , const vec3e& v1 , const vec3e& v2 ){
	vec3e xBasis = ( v1 - v0 ).normalized();
	vec3e vecB   = ( v2 - v0 ).normalized();
	return xBasis.cross( vecB ).normalized(); // This should already be normalized
}

vec3e get_CCW_tri_norm( const matXe& V ){
	vec3e v0 = V.row(0);
	vec3e v1 = V.row(1);
	vec3e v2 = V.row(2);
	return get_CCW_tri_norm( v0 , v1 , v2 );
}

matXe N_from_VF( const matXe& V , const matXi& F ){
	size_t len = F.rows();
	matXe allNorms = matXe::Zero( len , 3 );
	vec3e v0 , v1 , v2;
	for( size_t i = 0 ; i < len ; i++ ){
		v0 = V.row( F( i , 0 ) );
		v1 = V.row( F( i , 1 ) );
		v2 = V.row( F( i , 2 ) );
		allNorms.row( i ) = get_CCW_tri_norm( v0 , v1 , v2 );
	}
	return allNorms;
}

// __ End Mesh __

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/


#ifndef OGL_GEO_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_GEO_H // from multiple imports

#include <array>
using std::array;
#include <vector>
using std::vector;

#include "OGL_utils.hpp"

typedef array<float,2> vec2f;
typedef array<float,3> vec3f;

vec2f polr_2_cart_0Y( const vec2f& polarCoords ){ // 0 angle is +Y North 
    // Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ 
    return vec2f{ polarCoords[0] * sinf( polarCoords[1] ) , polarCoords[0] * cosf( polarCoords[1] ) };
}

vector<vec2f> circ_space( float dia , uint numPts , const vec2f& center ){
    // Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified 
    float div = 2.0 * M_PI / numPts;
    vector<vec2f> circPts;
    vec2f offset;
    for( uint pntDex = 0 ; pntDex < numPts ; pntDex++ ){
		offset = polr_2_cart_0Y( vec2f{ dia/2 , pntDex * div } );
		offset[0] = center[0] + offset[0];
		offset[1] = center[1] + offset[1];
		circPts.push_back( offset );
	}
    return circPts;
}

vector<vec3f> pts_XY_at_Z( const vector<vec2f>& XY , float Z ){
	// Set 'XY' points in 3D space at 'Z' height
	vector<vec3f> rtnPts;
	uint len = XY.size();
	for( uint i = 0 ; i < len ; i++ ){  rtnPts.push_back( vec3f{ XY[i][0], XY[i][1], Z } );  }
	return rtnPts;
}

void glVec3f( const vec3f& v ){  glVertex3f( v[0] , v[1] , v[2] );  }
void glClr3f( const vec3f& c ){  glColor3f( c[0] , c[1] , c[2] );  }

void draw_cylinder( const vec3f& origin , float length , float radius , uint facets ,
					const vec3f& fillClr , const vec3f& lineClr ){
	// Draw a cylinder of 'length' and 'radius' with the center of one end at 'origin', with an 'axisDir'
	
	// 0. Init
	float radMargin = 1.0 + 2/200.0;
	float lineWidth = length / 100.0;
	glLineWidth( 2.0 );
	
	// 1. Get cicle coordinates
	vector<vec2f> baseCirc = circ_space( radius * 2.0             , facets , vec2f{ origin[0] , origin[1] } );
	vector<vec2f> lineCirc = circ_space( radius * 2.0 * radMargin , facets , vec2f{ origin[0] , origin[1] } );
	// 2. Get bottom circle
	vector<vec3f> btmCirc   = pts_XY_at_Z( baseCirc , origin[2] );
	vector<vec3f> btmLinOut = pts_XY_at_Z( lineCirc , origin[2] );
	vector<vec3f> btmLineIn = pts_XY_at_Z( lineCirc , origin[2] + lineWidth );
	// 3. Get top circle
	vector<vec3f> topCirc   = pts_XY_at_Z( baseCirc , origin[2] + length );
	vector<vec3f> topLinOut = pts_XY_at_Z( lineCirc , origin[2] + length);
	vector<vec3f> topLineIn = pts_XY_at_Z( lineCirc , origin[2] + length - lineWidth );
	// 3.5. Set color
	glClr3f( fillClr );
	// 4. Paint barrel
	glBegin( GL_QUAD_STRIP );
	uint len = baseCirc.size();
	uint i = 0;
	for( uint i = 0 ; i < len ; i++ ){
		glVec3f( btmCirc[i] );
		glVec3f( topCirc[i] );
	}
	glVec3f( btmCirc[0] );
	glVec3f( topCirc[0] );
	glEnd();
	// 5. Paint top
	glBegin( GL_TRIANGLE_FAN );
	for( i = 0 ; i < len - 1 ; i++ ){  glVec3f( topCirc[i] );  }
	glEnd();
	// 6. Paint bottom
	glBegin( GL_TRIANGLE_FAN );
	for( i = 0 ; i < len - 1 ; i++ ){  glVec3f( btmCirc[i] );  }
	glEnd();
	// 7. Set line color
	glClr3f( lineClr );
	
	vec3f last;
	// 8. Paint top border
	glBegin( GL_LINES );
	last = topLineIn[0];
	for( uint i = 1 ; i < len ; i++ ){
		glVec3f( last );
		glVec3f( topLineIn[i] );
		last = topLineIn[i];
	}
	glVec3f( last );
	glVec3f( topLineIn[0] );
	glEnd();
	// 9. Paint bottom border
	glBegin( GL_LINES );
	last = btmLineIn[0];
	for( uint i = 1 ; i < len ; i++ ){
		glVec3f( last );
		glVec3f( btmLineIn[i] );
		last = btmLineIn[i];
	}
	glVec3f( last );
	glVec3f( btmLineIn[0] );
	glEnd();
}

vec3f sphr_2_cart_pnt( float r , float th , float ph ){
	// For the given spherical coordinates , Return the Cartesian point
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	return vec3f{
		r * Sinf( th ) * Cosf( ph ) , 
		r * Sinf( ph ) , 
		r * Cosf( th ) * Cosf( ph )
	};
}

vec3f sample_from_AABB( float bbox[2][3] ){
	// Sample uniformly from an Axis Aligned Bounding Box
	return vec3f{ (float) randrange( bbox[0][0] , bbox[1][0] ) , 
				  (float) randrange( bbox[0][1] , bbox[1][1] ) , 
				  (float) randrange( bbox[0][2] , bbox[1][2] ) };
}

vec3f vec_sphr( float r , float th , float ps ){
	// Return a vertex in spherical coordinates , Theta axis is Z+
	// Based on code provided by Willem A. (Vlakkies) Schreüder  
	return vec3f{ r * Cosf( th ) * Cosf( ps ) , 
				  r * Sinf( th ) * Cosf( ps ) , 
				  r * Sinf( ps ) };
}

#endif
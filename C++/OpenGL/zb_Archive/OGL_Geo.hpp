#ifndef OGL_GEO_H // This pattern is to prevent symbols to be loaded multiple times
#define OGL_GEO_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports /////

#include <array>
using std::array;
#include <vector>
using std::vector;

#include "OGL_utils.hpp"





////////// GEOMETRY HELPERS ////////////////////////////////////////////////////////////////////////

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

vec3f sphr_2_cart_pnt( float r , float th , float ph ){
	// For the given spherical coordinates , Return the Cartesian point
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	return vec3f{
		r * Sinf( th ) * Cosf( ph ) , 
		r * Sinf( ph ) , 
		r * Cosf( th ) * Cosf( ph )
	};
}

vec3f sample_from_AABB_f( float bbox[2][3] ){
	// Sample uniformly from an Axis Aligned Bounding Box
	return vec3f{ randf( bbox[0][0] , bbox[1][0] ) , 
				  randf( bbox[0][1] , bbox[1][1] ) , 
				  randf( bbox[0][2] , bbox[1][2] ) };
}

vec3f vec_sphr( float r , float th , float ps ){
	// Return a vertex in spherical coordinates , Theta axis is Z+
	// Based on code provided by Willem A. (Vlakkies) Schreüder  
	return vec3f{ r * Cosf( th ) * Cosf( ps ) , 
				  r * Sinf( th ) * Cosf( ps ) , 
				  r * Sinf( ps ) };
}

void Vertex_sphr( float th , float ph ){
	// Draw vertex in polar coordinates
	// Author: Willem A. (Vlakkies) Schreüder  
	// glColor3f( Cos( th )*Cos( th ) , Sin(ph)*Sin(ph) , Sin(th)*Sin(th));
	glVertex3d( Sinf( th ) * Cosf( ph ) , 
				Sinf( ph ) , 
				Cosf( th ) * Cosf( ph ) );
}

// void push_transform( float tX, float tY, float tZ, float rX, float rY, float rZ ){
// 	// Push a transformation onto the matrix stack, NOTE: Should be followed by `glPopMatrix()`
// 	glPushMatrix();
// 	/// Rotate ///
// 	glRotated( (double) rX, 1.0, 0.0, 0.0 ); // 1. Rotate around the X axis
// 	glRotated( (double) rY, 0.0, 1.0, 0.0 ); // 2. Rotate around the Y axis
// 	glRotated( (double) rZ, 0.0, 0.0, 1.0 ); // 3. Rotate around the Z axis
// 	/// Translate ///
// 	glTranslated( (double) tX, (double) tY, (double) tZ );
// }

// void push_transform( const vec3f& t, const vec3f& r ){
// 	// Push a transformation onto the matrix stack, NOTE: Should be followed by `glPopMatrix()`
// 	push_transform( t[0], t[1], t[2], r[0], r[1], r[2] );
// }

////////// GEOMETRY RENDERING //////////////////////////////////////////////////////////////////////


void draw_cylinder_lined( const vec3f& origin , float length , float radius , uint facets ,
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


void draw_sphere_2( float x , float y , float z , float r ){
	// Draw a sphere (version 2) at (x,y,z) radius (r)
	// Author: Willem A. (Vlakkies) Schreüder  
	const int d = 5;
	int       th , ph;

	//  Save transformation
	glPushMatrix();
	//  Offset and scale
	glTranslated( x , y , z );
	glScaled( r , r , r );

	//  Latitude bands
	for( ph = -90 ; ph < 90 ; ph += d ){
		glBegin( GL_QUAD_STRIP );
		for( th = 0 ; th <= 360 ; th += d ){
			Vertex_sphr( th , ph     );
			Vertex_sphr( th , ph + d );
		}
		glEnd();
	}

	//  Undo transformations
	glPopMatrix();
}


void draw_cuboid_lined( float x , float y , float z ,
						float dx , float dy , float dz ,
						float fillColor[3] , float lineColor[3] ){
	// Draw a cube at (x,y,z) dimensions (dx,dy,dz) 
	float lineOffset = 1.005;
	//  Save transformation
	glPushMatrix();
	glColor3f( fillColor[0] , fillColor[1] , fillColor[2] );
	//  Offset
	glTranslated( x , y , z );
	glScaled( dx/2.0f , dy/2.0f , dz/2.0f );
	
	//  Cube
	glBegin(GL_QUADS);
		//  Front
		glVertex3f(-1,-1, 1);
		glVertex3f(+1,-1, 1);
		glVertex3f(+1,+1, 1);
		glVertex3f(-1,+1, 1);
		//  Back
		glVertex3f(+1,-1,-1);
		glVertex3f(-1,-1,-1);
		glVertex3f(-1,+1,-1);
		glVertex3f(+1,+1,-1);
		//  Right
		glVertex3f(+1,-1,+1);
		glVertex3f(+1,-1,-1);
		glVertex3f(+1,+1,-1);
		glVertex3f(+1,+1,+1);
		//  Left
		glVertex3f(-1,-1,-1);
		glVertex3f(-1,-1,+1);
		glVertex3f(-1,+1,+1);
		glVertex3f(-1,+1,-1);
		//  Top
		glVertex3f(-1,+1,+1);
		glVertex3f(+1,+1,+1);
		glVertex3f(+1,+1,-1);
		glVertex3f(-1,+1,-1);
		//  Bottom
		glVertex3f(-1,-1,-1);
		glVertex3f(+1,-1,-1);
		glVertex3f(+1,-1,+1);
		glVertex3f(-1,-1,+1);
	//  End
	glEnd();
	
	// Draw outline
	float d = lineOffset;
	glColor3f( lineColor[0] , lineColor[1] , lineColor[2] );
	
	glBegin( GL_LINES );
		// Bottom
		glVertex3f(-d,-d,-d);
		glVertex3f(+d,-d,-d);
		
		glVertex3f(+d,-d,-d);
		glVertex3f(+d,+d,-d);
		
		glVertex3f(+d,+d,-d);
		glVertex3f(-d,+d,-d);
		
		glVertex3f(-d,+d,-d);
		glVertex3f(-d,-d,-d);

		// Top
		glVertex3f(-d,-d,+d);
		glVertex3f(+d,-d,+d);
		
		glVertex3f(+d,-d,+d);
		glVertex3f(+d,+d,+d);
		
		glVertex3f(+d,+d,+d);
		glVertex3f(-d,+d,+d);
		
		glVertex3f(-d,+d,+d);
		glVertex3f(-d,-d,+d);
		
		// Sides
		glVertex3f(-d,-d,+d);
		glVertex3f(-d,-d,-d);
		
		glVertex3f(+d,-d,+d);
		glVertex3f(+d,-d,-d);
		
		glVertex3f(+d,+d,+d);
		glVertex3f(+d,+d,-d);
		
		glVertex3f(-d,+d,+d);
		glVertex3f(-d,+d,-d);
		
	glEnd();
	//  Undo transformations
	glPopMatrix();
}

#endif
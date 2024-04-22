/*****************************
 HW3.cpp
 James Watson, 2018 September
 Display a complex scene
 ****************************/ 
/*
~~ DEV PLAN ~~
[Y] Convert HW2 to C++
[Y] Gut old program
[Y] Include cpp_helpers
[Y] Simplest vector lib, Templated
[Y] Cylinder (closed) function & test
[Y] Build Link 1
[Y] Rotate link 1
[Y] Build Base
[Y] Build all Distal Links
	[Y] Link 2
	[Y] Link 3
	[Y] Link 4
	[Y] Link 5
	[Y] Link 6
{Y} Grid floor
[ ] Toggle Manual control
[ ] Manual control of 6 joints
{ } Gripper?
*/

// === INIT ================================================================================================================================

// === Includes & Defines ===

// ~~ Standard ~~
#include <stdio.h> // Streams to communicate with devices such as keyboards, printers, terminals or with any other type of files supported 
#include <stdlib.h> // defines four variable types, several macros, and various functions for performing general functions. , size_t
#include <stdarg.h> // macros to access individual args of a list of unnamed arguments whose number and types are not known to the called function

// ~~ OpenGL ~~

// OpenGL with prototypes for glext
// NOTE: This MUST appear before '#include <GL/glext.h>' , otherwise will not work!
#define GL_GLEXT_PROTOTYPES // Important for all of your 5229 programs

// ~~ System-Specific Includes ~~
#ifdef __APPLE__ // This constant is always defined on Apple machines
      #include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
	#include <GL/glut.h>
	#include <GL/glext.h>	
#endif

// ~~ Local ~~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/
#include "OGL_utils.h" // _ Utility functions for 5229
#include "nd_vector.h" // _ A templated header library for extremely simple vector operations

// ~~ Namespace Import ~~
using namespace ndv;
using vec2f = nd_vector<float,2>;
using vec3f = nd_vector<float,3>;

// ___ End Include ___

// === GLOBALS ===

// ~~ Assignment ~~
string HWname = "HW3";
// ~~ Data ~~

// ~~ Params ~~

// ~~ Control ~~
std::vector<float> q    = {   0,	 0,		 0,		0,	0,	0 };
std::vector<float> qDot = {  30.0 , 30.0 , 30.0 , 30.0 , 30.0 , 30.0}; // deg/sec
//~ std::vector<float> qDot = {  00.0 , 00.0 , 00.0 , 00.0 , 00.0 , 00.0}; // deg/sec
float DEGRINCR = 2.0;

enum CTRLMODE{ AUTO_CTRL , MANL_CTRL };
CTRLMODE CURRMODE = AUTO_CTRL;
enum JNTNUMBR{ JOINT1 , JOINT2 , JOINT3 , JOINT4 , JOINT5 , JOINT6 , NONE };
JNTNUMBR CURJOINT = NONE;

// ___ END GLOBAL ___

// ___ END INIT ____________________________________________________________________________________________________________________________


// === FUNCTIONS ===========================================================================================================================

vec2f polr_2_cart_0Y( const vec2f& polarCoords ){ // 0 angle is +Y North 
    // Convert polar coordinates [radius , angle (radians)] to cartesian [x , y]. Theta = 0 is UP = Y+ 
    return vec2f{ polarCoords[0] * sinf( polarCoords[1] ) , polarCoords[0] * cosf( polarCoords[1] ) };
}

std::vector<vec2f> circ_space( float dia , uint numPts , const vec2f& center ){
    // Return a list of 'numPts' points equally spaced around a 2D circle with a center at (0,0), or at 'center' if specified 
    float div = 2.0 * M_PI / numPts;
    std::vector<vec2f> circPts;
    vec2f offset;
    for( uint pntDex = 0 ; pntDex < numPts ; pntDex++ ){
		offset = polr_2_cart_0Y( vec2f{ dia/2 , pntDex * div } );
		offset = center + offset;
		circPts.push_back( offset );
	}
    return circPts;
}

std::vector<vec3f> pts_XY_at_Z( const std::vector<vec2f>& XY , float Z ){
	// Set 'XY' points in 3D space at 'Z' height
	std::vector<vec3f> rtnPts;
	uint len = XY.size();
	for( uint i = 0 ; i < len ; i++ ){  rtnPts.push_back( vec3f{ XY[i] , Z } );  }
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
	std::vector<vec2f> baseCirc = circ_space( radius * 2.0             , facets , vec2f{ origin[0] , origin[1] } );
	std::vector<vec2f> lineCirc = circ_space( radius * 2.0 * radMargin , facets , vec2f{ origin[0] , origin[1] } );
	// 2. Get bottom circle
	std::vector<vec3f> btmCirc   = pts_XY_at_Z( baseCirc , origin[2] );
	std::vector<vec3f> btmLinOut = pts_XY_at_Z( lineCirc , origin[2] );
	std::vector<vec3f> btmLineIn = pts_XY_at_Z( lineCirc , origin[2] + lineWidth );
	// 3. Get top circle
	std::vector<vec3f> topCirc   = pts_XY_at_Z( baseCirc , origin[2] + length );
	std::vector<vec3f> topLinOut = pts_XY_at_Z( lineCirc , origin[2] + length);
	std::vector<vec3f> topLineIn = pts_XY_at_Z( lineCirc , origin[2] + length - lineWidth );
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
	return vec3f{ randrange( bbox[0][0] , bbox[1][0] ) , 
				  randrange( bbox[0][1] , bbox[1][1] ) , 
				  randrange( bbox[0][2] , bbox[1][2] ) };
}

// ___ END FUNC ____________________________________________________________________________________________________________________________


// === CLASSES =============================================================================================================================

float AXESSCALE = 0.17;

// == struct DH_Parameters ==
struct DH_Parameters{
	std::vector<float> alpha; 
	std::vector<float> a; 
	std::vector<float> d; 
	std::vector<float> theta; 
};

// __ End DH __


// == class RobotLink ==

class RobotLink{
public:
		
	// ~~ Con/Destructors ~~
	RobotLink( float pTheta , const vec3f& pOrigin , 
			   float pD_dist , float pA_dist , const vec3f& pNextRotnAxis , float pNextRotnAngl , 
			   void (*pDrawFunc)() );

	// ~~ Configuration ~~
	void add_distal( RobotLink* link );

	// ~~ Robot Motion ~~
	void set_theta( float pTheta );
	
	// ~~ Drawing ~~
	void draw();

protected:
	// ~ Proximal Joint ~
	float /* ----------- */ theta;
	// ~ Relative Location ~
	vec3f /* ----------- */ origin;
	// ~ Distal Joint ~
	float /* ----------- */ d_dist;
	float /* ----------- */ a_dist;
	vec3f /* ----------- */ nextRotnAxis;
	float /* ----------- */ nextRotnAngl;
	// ~ Distal Links ~
	std::vector<RobotLink*> distalLinks;
	// ~ Rendering ~
	void /* ------------ */ (*drawFunc)();
	// ~ Bookkeeping ~
	uint /* ------------ */ index;
};

RobotLink::RobotLink( float pTheta , const vec3f& pOrigin , 
					  float pD_dist , float pA_dist , const vec3f& pNextRotnAxis , float pNextRotnAngl , 
					  void (*pDrawFunc)() ){
	// Create a robot link with an orientation, origin, and draw function
	theta        = pTheta;
	origin       = pOrigin;
	d_dist       = pD_dist;
	a_dist       = pA_dist;
	nextRotnAxis = pNextRotnAxis;
	nextRotnAngl = pNextRotnAngl;
	drawFunc     = pDrawFunc;
	
	cout << "Created a RobotLink with: alpha = " << nextRotnAngl << " , a = " << a_dist << " , d = " << d_dist << " , theta = " << theta << endl;
}

void RobotLink::add_distal( RobotLink* link ){  distalLinks.push_back( link );  }

void RobotLink::set_theta( float pTheta ){  theta = pTheta;  } // Set the angle of the joint at the base of the link

void RobotLink::draw(){
	// Render the link and all distal links
	// 1. Save transformation for this link
	glPushMatrix(); // Transformations apply only to this link
	
	
	
	// 2. Transform link
	glRotated( theta , 0 , 0 , 1 ); // ------------------ 2 , end
	glTranslated( origin[0] , origin[1] , origin[2] ); // 1 , bgn 
	// 3. Render link
	drawFunc();
	// 4. Downstream link transform
	
	
	
	glTranslated( a_dist , 0 , 0 ); // ------------------------- 1 , bgn 
	glTranslated( 0 , 0 , d_dist ); // ------------------------- 1 , bgn 

	glRotated( nextRotnAngl , // ------------------------------------ 2 , end
			   nextRotnAxis[0] , nextRotnAxis[1] , nextRotnAxis[2] );
	
	
	
	
	draw_origin( AXESSCALE );
	
	// 4. For each distal link
	uint numDistl = distalLinks.size();
	// 5. Draw the link *relative* to this link!
	for( uint i = 0 ; i < numDistl ; i++ ){  distalLinks[i]->draw();  }
	// 6. Untransform from link frame
	glPopMatrix();
}

// __ End RobotLink __


// == Link Draw Funcs ==

/*
* Base: Dia - 0.145 , Height - 0.015
draw_cylinder( vec3f{ 0 , 0 , ( DH.d[1] - length/2.0f ) + zOffset } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
* Link 1:
	d = 0.070
	Vertical   , Dia - 0.120 , Height - 0.140
	Horizontal , Dia - 0.120 , Height - 0.070
*/

uint NUMCYLFACETS = 75;
vec3f URGREY{ 117/255.0 , 125/255.0 , 130/255.0 };
vec3f URBLCK{  50/255.0 ,  50/255.0 ,  50/255.0 };
vec3f URMETL{ 224/255.0 , 224/255.0 , 224/255.0 };

// == Robot Parameters ==
// Joint:                    0 ,    1      ,  2     ,  3     ,  4      ,   5      ,  6
std::vector<float> alpha = { 0 ,   90      ,  0     ,  0     , 90      , -90      ,  0      }; 
std::vector<float> a     = { 0 , 	0      , -0.425 , -0.392 ,  0      ,   0      ,  0      }; 
std::vector<float> d     = { 0 ,	0.0892 ,  0     ,  0     ,  0.1093 ,   0.0948 ,  0.0825 }; 
std::vector<float> theta = { 0 ,    0      ,  0     ,  0     ,  0      ,   0      ,  0      }; 
DH_Parameters DH{ alpha , a , d , theta };
// __ End Params __

float BASEHEIGHT = 0.017;

void lnk1_draw(){
	// Draw Link 1
	float length   = 0.140;
	float diameter = 0.120;
	float zOffset  = 0.015;
	glPushMatrix(); // Transformations apply only to this link
	draw_cylinder( vec3f{ 0 , 0 , ( DH.d[1] - length/2.0f ) } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	
	glTranslated( 0 , 0 , length/2.0 + zOffset ); 
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3f{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	glPopMatrix();
}

void lnk2_draw(){
	// Draw Link 2
	float length   = 0.140;
	float diameter = 0.120;
	float armDia   = 0.085;
	glPushMatrix(); // Transformations apply only to this link
	// 1. Vertical cylinder
	glRotated( -90 , 0 , 0 , 1 ); 
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3f{ 0,0,0 } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	// 2. Horizontal Cylinders
	
	
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , length/2.0 , 0 ); 
	
	draw_cylinder( vec3f{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	draw_cylinder( vec3f{ 0,0,diameter/2.0f } , 0.295 , armDia/2.0f , NUMCYLFACETS , 
				   URMETL , URBLCK );
	draw_cylinder( vec3f{ 0,0,-DH.a[2]-length/2.0f } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3f{ 0,-DH.a[2],-length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	glPopMatrix();
}

void lnk3_draw(){
	// Draw Link 3
	float length   = 0.090;
	//~ float bigLen   = 0.140;
	float diameter = 0.072;
	float armDia   = diameter;
	glPushMatrix(); // Transformations apply only to this link
	// 1. Vertical cylinder
	glRotated( -90 , 0 , 0 , 1 ); 
	glTranslated( 0 , 0 , - length * 0.5f ); 
	
	float xtraHght = 0.020;
	float zAdjust  = 0.002;
	// - bigLen/2.0f - length/2.0f - length
	draw_cylinder( vec3f{ 0,0, DH.d[3] + zAdjust } , length + xtraHght , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	// 2. Horizontal Cylinders
	
	
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , length/2.0f , 0.0 ); 
	
	draw_cylinder( vec3f{ 0,0,0 } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	draw_cylinder( vec3f{ 0,0,0.045 } , -DH.a[3]-length , armDia/2.0f , NUMCYLFACETS , 
				   URMETL , URBLCK );
	draw_cylinder( vec3f{ 0,0,-DH.a[3]-length/2.0f } , length/2.0f , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	glRotated( 90 , 1 , 0 , 0 ); 
	draw_cylinder( vec3f{ 0 , -DH.a[3] , -length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	glPopMatrix();
}

void lnk4_draw(){
	// Draw Link 4
	float length   = 0.090;
	float diameter = 0.072;
	
	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3f{ 0,0, 0.00 } , DH.d[4]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
				   
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , DH.d[4]-length/2.0f , 0.0 ); 
	// - ( DH.d[3] -length/2.0f )
	// DH.d[3] + length/2.0f + ( DH.d[3] -length/2.0f )
	
	draw_cylinder( vec3f{ 0,0, -length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	
	glPopMatrix();
}

void lnk5_draw(){
	// Draw Link 5
	float length   = 0.090;
	float diameter = 0.072;
	
	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3f{ 0,0, 0.00 } , DH.d[5]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
				   
	glRotated( 90 , 1 , 0 , 0 ); 
	glTranslated( 0 , DH.d[5]-length/2.0f , 0.0 ); 
	// - ( DH.d[3] -length/2.0f )
	// DH.d[3] + length/2.0f + ( DH.d[3] -length/2.0f )
	
	draw_cylinder( vec3f{ 0,0, -length/2.0f } , length , diameter/2.0f , NUMCYLFACETS , 
				   URGREY , URBLCK );
	
	glPopMatrix();
}

void lnk6_draw(){
	// Draw Link 6
	float length   = 0.090;
	float diameter = 0.072;
	glPushMatrix(); // Transformations apply only to this link
	glTranslated( 0 , 0 , length/2.0f ); 
	
	draw_cylinder( vec3f{ 0,0, 0.00 } , DH.d[6]-length/2.0f  , diameter/2.0f , NUMCYLFACETS , 
				   URMETL , URBLCK );
	glPopMatrix();
}

// __ End Link Draw __


// == class PegBlock ==

class PegBlock{
	// A LEGO brick for space children
public:

	PegBlock( float minSide , float maxSide , float bbox[2][3] );
	
	void draw();

protected:
	vec3f center;
	float th;
	float ph;
	float side; // This is the scale
	std::vector<float> pegLen;
	vec3f fillColor;
	vec3f lineColor;
};

PegBlock::PegBlock( float minSide , float maxSide , float bbox[2][3] ){
	// Generate and instantiate a cube with pegs sticking out of it
	// 1. Choose a side length
	side = randrange( minSide , maxSide );
	// 2. Choose a center in the bbox
	center = sample_from_AABB( bbox );
	// 3. Choose theta and phi
	th = 360 * rand_float();
	ph = 180 * rand_float();
	// 4. Choose peg lengths
	pegLen = randrange_vec( side * 0.1f , side , (size_t)4 * 6 );
	// 5. Choose colors
	fillColor = rand_3<float>();
	lineColor = vec3f{ 0,0,0 };
	
	cout << "Created a PegBlock! With fill color " << fillColor << " and line color " << lineColor << endl;
}

void PegBlock::draw(){
	// Draw pegblock
	glPushMatrix();
	// 1. Rotate into theta and phi
	glRotated( ph , 1 , 0 , 0 ); // 2. Rotate around the X axis
	glRotated( th , 0 , 0 , 1 ); // 1. Rotate around the Z axis
	glTranslated( center[0] , center[1] , center[2] );
	// 2. Draw cube
	float fill[3];	fillColor.load_array( fill );
	float line[3];	lineColor.load_array( line );
	uint pegDex = 0;
	cube( 0 , 0 , 0 ,
          side , side , side ,
          fill , line );
	// 3. For each cube face
		// 4. Rotate into face
		
		// 5. Draw 4 pegs
		draw_cylinder( vec3f{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		// 6. Pop
		
		glRotated( 180 , 1 , 0 , 0 ); // 2. Rotate around the X axis
		// 5. Draw 4 pegs
		draw_cylinder( vec3f{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
					   
		glRotated( -90 , 1 , 0 , 0 ); // 2. Rotate around the X axis
		// 5. Draw 4 pegs
		draw_cylinder( vec3f{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
		draw_cylinder( vec3f{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
					   fillColor , lineColor );  pegDex++;
					   
		for( uint i = 0 ; i < 3 ; i++ ){
			glRotated( -90 , 0 , 1 , 0 ); // 2. Rotate around the X axis
			// 5. Draw 4 pegs
			draw_cylinder( vec3f{ side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   fillColor , lineColor );  pegDex++;
			draw_cylinder( vec3f{ -side/4.0f , side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   fillColor , lineColor );  pegDex++;
			draw_cylinder( vec3f{ side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   fillColor , lineColor );  pegDex++;
			draw_cylinder( vec3f{ -side/4.0f , -side/4.0f , side/2.0f }  , pegLen[ pegDex ] , side/8.0 , NUMCYLFACETS ,
						   fillColor , lineColor );  pegDex++;
		}
		
	
	// 7. Pop
	glPopMatrix();
}

// __ End PegBlock __


// ___ END CLASS ___________________________________________________________________________________________________________________________


// === VARIABLES ===========================================================================================================================

int DFLT_THETA = -205;
int DFLT_PSI   =  -50;

// ~~ Globals ~~
int    th     = DFLT_THETA; //_ Azimuth of view angle
int    ps     = DFLT_PSI; //_ Elevation of view angle
float  dim    =   1; // Dimension of orthogonal box


//~ float pTheta , const vec3f& pOrigin , 
					  //~ float pD_dist , float pA_dist , const vec3f& pNextRotnAxis , float pNextRotnAngl , 
					  //~ void (*pDrawFunc)()



// ~~ Robot ~~

vec3f X{1,0,0};

// ~ Create Links ~
RobotLink Link1{ 0.0 , vec3f{ 0 , 0 , 0.000 } , 
				 DH.d[1] , DH.a[1] , X ,  DH.alpha[1] , 
				 lnk1_draw };
				

RobotLink Link2{ 0.0 , vec3f{ 0 , 0.000 , 0.000 } , 
				 DH.d[2] , DH.a[2] , X ,  DH.alpha[2] , // 3.0*0.045 + 0.280
				 lnk2_draw };
				 
RobotLink Link3{ 0.0 , vec3f{ 0 , 0 , 0.000 } , 
				DH.d[3] , DH.a[3] , X ,  DH.alpha[3] , 
				 lnk3_draw };
				 
RobotLink Link4{ 0.0 , vec3f{ 0 , 0 , 0.000 } , 
				DH.d[4] , DH.a[4] , X ,  DH.alpha[4] , 
				 lnk4_draw };
					
RobotLink Link5{ 0.0 , vec3f{ 0 , 0 , 0.000 } , 
				DH.d[5] , DH.a[5] , X ,  DH.alpha[5] , 
				 lnk5_draw };
				 
RobotLink Link6{ 0.0 , vec3f{ 0 , 0 , 0.000 } , 
				DH.d[6] , DH.a[6] , X ,  DH.alpha[6] , 
				 lnk6_draw };

// ~~ Silly Things ~~

uint numBlocks = 10;
std::vector<PegBlock*> toys;

float minSide = 0.050 , 
	  maxSide = 0.200 ;
float bbox[2][3] = {
	{ -dim , -dim , -dim } , 
	{  dim ,  dim , 0.0f  }
};


// ___ END VAR _____________________________________________________________________________________________________________________________


// === DRAWING =============================================================================================================================

/*
 *  Display the scene
 */
void display(){
	
	//  Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	// ==== Redraw ====
	
	//  Set view angle
	glRotated( ps , 1 , 0 , 0 ); // 2. Rotate around the X axis
	glRotated( th , 0 , 0 , 1 ); // 1. Rotate around the Z axis
	
	//~ draw_origin( dim / 3.0 );
	draw_grid_org_XY( 0.250 , 20 , 20 , 0.5 );
	
	// Draw the base
	//Base: Dia - 0.145 , Height - 0.015
	draw_cylinder( vec3f{ 0 , 0 , 0 } , BASEHEIGHT , 0.145/2.0f , NUMCYLFACETS , 
				   URMETL , URBLCK );
	
	if(0){
		draw_cylinder( vec3f{ 4,4,4 } , 10.0 , 2.5 , 50 , 
					   vec3f{ 0,0,1 } , vec3f{ 1,1,1 } );
	}else{
		
		Link1.draw(); // This will draw all distal links as well
		
	}
	
	for( uint i = 0 ; i < numBlocks ; i++ ){
		toys[i]->draw();
	}


	// NOTE: Text color MUST be specified before raster position for bitmap text
	// https://www.opengl.org/archives/resources/features/KilgardTechniques/oglpitfall/
	
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	
	//  Display status
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	
	string editStr = ", Edit: " + to_string( (uint)CURJOINT + (uint)1 );
	
	Print( " q = { %2.1f , %2.1f , %2.1f , %2.1f , %2.1f , %2.1f } | Mode: %s %s |  theta %i , psi %i" , 
		   q[0] , q[1] , q[2] , q[3] , q[4] , q[5] , 
		   CURRMODE == AUTO_CTRL ? "AUTO" : "MANUAL" , 
		   CURRMODE == MANL_CTRL && CURJOINT != NONE ? editStr.c_str() : "" ,
		   th , ps );

	// ____ End Draw ____

	//  Flush and swap
	glFlush();
	glutSwapBuffers();
}

// ___ END DRAW ____________________________________________________________________________________________________________________________


// === INTERACTION =========================================================================================================================



void key( unsigned char ch , int x , int y ){
	// GLUT calls this routine when a key is pressed
	//  Exit on ESC
	
	switch( ch ){
		
		// ~~ Rotate Ctrl ~~
		
		case 27 : // [Esc] : Exit
			exit( 0 );
			break;
			
		case '0' : // 0 : Set view angles to 0
			th = DFLT_THETA;
			ps = DFLT_PSI;
			printf( "theta and psi reset!\n" );
			break;
		
		// ~~ Program Controls ~~
		
		case 'c' : // Toggle the control mode , AUTO / MANUAL
			if( CURRMODE == AUTO_CTRL )  CURRMODE = MANL_CTRL;  else  CURRMODE = AUTO_CTRL;
		
		// ~ Choose a joint to edit in manual drive mode ~
		
		case '1' : // Select joint 1
			if( CURRMODE == MANL_CTRL )  CURJOINT = JOINT1;
			break;
		case '2' : // Select joint 2 
			if( CURRMODE == MANL_CTRL )  CURJOINT = JOINT2;
			break;
		case '3' : // Select joint 3
			if( CURRMODE == MANL_CTRL )  CURJOINT = JOINT3;
			break;
		case '4' : // Select joint 4
			if( CURRMODE == MANL_CTRL )  CURJOINT = JOINT4;
			break;
		case '5' : // Select joint 5
			if( CURRMODE == MANL_CTRL )  CURJOINT = JOINT5;
			break;
		case '6' : // Select joint 6
			if( CURRMODE == MANL_CTRL )  CURJOINT = JOINT6;
			break;
		case 'n' : // Select NO joint
			if( CURRMODE == MANL_CTRL )  CURJOINT = NONE;
			break;
		
		// ~ Edit Joint Angles ~
		case 'z' : // Zero-angle position
			if( CURRMODE == MANL_CTRL )  q = { 0,0,0,0,0,0 };
			break;
		case '-' : // Decrement the chosen joint
			if( CURJOINT != NONE ) q[CURJOINT] -= DEGRINCR;
			break;
		case '=' : // Increment the chosen joint 
			if( CURJOINT != NONE ) q[CURJOINT] += DEGRINCR;
			break;
			
		// <?> : Keys are nice, I guess!
			
		default :
			printf( "There is no function for this key!\n" );
		
	}
	
	//  Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}

void special( int key , int x , int y ){
	// GLUT calls this routine when an arrow key is pressed
	//  Right arrow key - increase azimuth by 5 degrees
	if( key == GLUT_KEY_RIGHT )
		th += 5;
	//  Left arrow key - decrease azimuth by 5 degrees
	else if( key == GLUT_KEY_LEFT )
		th -= 5;
	//  Up arrow key - increase elevation by 5 degrees
	else if( key == GLUT_KEY_UP )
		ps += 5;
	//  Down arrow key - decrease elevation by 5 degrees
	else if ( key == GLUT_KEY_DOWN )
		ps -= 5;
	//  Keep angles to +/-360 degrees
	th %= 360;
	ps %= 360;
	//  Tell GLUT it is necessary to redisplay the scene
	glutPostRedisplay();
}

void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
	//  Ratio of the width to the height of the window
	double w2h = ( height > 0 ) ? (double) width / height : 1;
	//  Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	//  Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	//  Orthogonal projection box adjusted for the
	//  aspect ratio of the window
	glOrtho( -dim * w2h , +dim * w2h , 
			 -dim       , +dim       , 
			 -dim       , +dim       );
	//  Switch to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	//  Undo previous transformations
	glLoadIdentity();
}

// ___ END INTERACT ________________________________________________________________________________________________________________________


// === SIMULATION ==========================================================================================================================

double _time_elapsed;
uint /* ------- */ DOF  = q.size();

void idle(){
	// 1. Get time
	_time_elapsed = glutGet( GLUT_ELAPSED_TIME ) / 1000.0;
	// 2. Increment q
	if( CURRMODE == AUTO_CTRL )
		for( uint i = 0 ; i < DOF ; i++ ){  q[i] = fmod( qDot[i] * _time_elapsed , 360 );  }
	// 3. Update joints
	Link1.set_theta( q[0] );
	Link2.set_theta( q[1] );
	Link3.set_theta( q[2] );
	Link4.set_theta( q[3] );
	Link5.set_theta( q[4] );
	Link6.set_theta( q[5] );
	// 3. Repaint
	glutPostRedisplay();
}

// ___ END SIM _____________________________________________________________________________________________________________________________


// === MAIN ================================================================================================================================

// Start up GLUT and tell it what to do
int main( int argc , char* argv[] ){
	/* initialize random seed: */
	srand (time(NULL));
	
	// == COMPONENT TESTS ==
	
	//~ for( uint i = 0 ; i < 10 ; i++ ){
		//~ cout << "Random Number: " << rand() << endl;
	//~ }
	
	// __ END TESTS __
	
	
	// ~~ Init Work ~~
	
	// ~ Connect Links ~
	Link1.add_distal( &Link2 );
	Link2.add_distal( &Link3 );
	Link3.add_distal( &Link4 );
	Link4.add_distal( &Link5 );
	Link5.add_distal( &Link6 );
		
	// ~ Draw boxes ~
	for( uint i = 0 ; i < numBlocks ; i++ ){
		toys.push_back( new PegBlock( minSide , maxSide , bbox ) );
	}
	
	
	//  Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	//  Request double buffered, true color window 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	
	//  Request 500 x 500 pixel window
	glutInitWindowSize( 975 , 725 );
	
	//  Create the window
	glutCreateWindow( ( "James Watson , " + HWname ).c_str() );
	
	//  Tell GLUT to call "idle" when there is nothing else to do
	glutIdleFunc(idle);
	
	glEnable( GL_DEPTH_TEST );
	//~ glDepthMask( GL_TRUE );
	//~ glDepthFunc( GL_LEQUAL );
	glDepthRange( 0.0f , 1.0f );
	
	//  Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );
	
	//  Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );
	
	//  Tell GLUT to call "special" when an arrow key is pressed
	glutSpecialFunc( special );
	
	//  Tell GLUT to call "key" when a key is pressed
	glutKeyboardFunc( key );
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	//  Return code
	return 0;
}

// ___ END MAIN ____________________________________________________________________________________________________________________________


/* === SPARE PARTS =========================================================================================================================

// == class Randocule ==

struct StickBall{
	// Branching molecular structure (without loops)
	vec3f origin;
	float stikLen;
	vec3f ballLoc; // relative spherical coords
	float ballRad;
	vec3f color;
	std::vector<StickBall*> nextBalls;
};

void erase_StickBall( StickBall* ball ){
	// Recursive Case: Branches
	uint len = ball->nextBalls.size();
	if( len > 0 ){
		for( uint i = 0 ; i < len ; i++ ){  erase_StickBall( ball->nextBalls[i] );  }
	// Base Case: No branches
	}else{
		delif( ball );
	}
}

class Randocule{
public:

	Randocule( float baseRadius , const vec3f& center , 
			   float minRadius , float maxRadius , float maxStickFactor , 
			   float branchProb , uint maxDepth );
			   
	void draw();

protected:

	void _generate( float baseRadius , const vec3f& center , 
					float minRadius , float maxRadius , float maxStickFactor , 
					float branchProb , uint maxDepth );

	vec3f rootCenter;
	std::vector<StickBall*> branches;
};

void Randocule::_generate( float baseRadius , const vec3f& center , 
						   float minRadius , float maxRadius , float maxStickFactor , 
						   float branchProb , uint maxDepth ){
	// Draw spheres connected by sticks with a random branching probability
	// This function assumes we are already in the proper reference frame 
	std::vector<float> radii;	float radius = 0.0f;
	std::vector<float> stLen;	float length = 0.0f;
	std::vector<vec3f> cntrs;
	// 1. Draw the base sphere
	sphere2( center[0] , center[1] , center[2] , baseRadius );
	// 2. While branching succeeds
	while(  dice_roll( (double)branchProb )  ){
		// 3. Define a sphere radius and append
		radius = randrange( minRadius , maxRadius );
		radii.push_back( radius );
		// 4. Define a stick length and append
		radius += baseRadius;
		length = randrange( radius , radius * maxStickFactor );
		stLen.push_back( length );
		// 5. Define spherical coordinates
		
	}
	// 6. For each branch
		// 7. Push matrix
		// 8. Recur
		// 9
}


// __ End Randocule __
   ___ END PARTS ___ */

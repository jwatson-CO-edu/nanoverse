/***********  
SOURCE_TEMPLATE.cpp
James Watson , YYYY MONTHNAME
A ONE-LINE DESRIPTION OF THE FILE

Template Version: 2018-06-07
***********/

#include "DH_Robot.h"

// === Classes and Structs =================================================================================================================

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
	
	
	// Capture the transform if we are at the final link
	if( is_leaf() ){  
		glGetFloatv( GL_MODELVIEW_MATRIX , OGLmat );  
		for( uint i = 0 ; i < 16 ; i++ ){  modelMat[i] = OGLmat[i];  }
	}
	
	
	// 4. For each distal link
	uint numDistl = distalLinks.size();
	// 5. Draw the link *relative* to this link!
	for( uint i = 0 ; i < numDistl ; i++ ){  distalLinks[i]->draw();  }
	// 6. Untransform from link frame
	glPopMatrix();
}

bool RobotLink::is_leaf(){  return distalLinks.size() == 0;  }

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

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

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/


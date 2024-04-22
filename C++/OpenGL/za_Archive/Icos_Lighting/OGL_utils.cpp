/***********  
OGL_utils.cpp
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

#include "OGL_utils.h"

// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

bool ErrCheck( const char* where ){
	// See if OpenGL has raised any errors
	// Author: Willem A. (Vlakkies) Schreüder  
	int err = glGetError();
	if( err ){  
		fprintf( stderr , "ERROR: %s [%s]\n" , gluErrorString( err ) , where );  
		return true;
	}else{  return false;  }
}

void Fatal( const char* format , ... ){
	// Scream and Run
	// Author: Willem A. (Vlakkies) Schreüder  
	va_list args;
	va_start( args , format );
	vfprintf( stderr , format , args );
	va_end( args );
	exit( 1 );
}

void glVec3e( const vec3e& v ){  glVertex3f( v[0] , v[1] , v[2] );  } // Set a vertex with an Eigen vector
void glNrm3e( const vec3e& n ){  glNormal3f( n[0] , n[1] , n[2] );  } // Set a normal with an Eigen vector
void glClr3e( const vec3e& c ){  glColor3f(  c[0] , c[1] , c[2] );  } // Set the color with an Eigen vector

void Print( const char* format , ... ){
	// Convenience routine to output raster text , Use VARARGS to make this more flexible   
	// Author: Willem A. (Vlakkies) Schreüder  
	char    buf[ LEN ];
	char*   ch = buf;
	va_list args;
	//  Turn the parameters into a character string
	va_start( args , format );
	vsnprintf( buf , LEN , format , args );
	va_end( args );
	//  Display the characters one at a time at the current raster position
	while( *ch ){
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18 , *ch++ );
	}
}

void Print( string format , ... ){
	// 'std::string' version of the above
	va_list args;
	va_start( args , format );
	Print( format.c_str() , args );
	va_end( args );
}

void draw_origin( float scale ){
	//  Draw scaled axes at origin in [R,G,B] for [X,Y,Z]
	glLineWidth( 1.5 );
	
	glBegin( GL_LINES );
		glColor3f( 1 , 0 , 0 ); // R
		glVertex3d( 0     , 0 , 0 ); // Bgn
		glVertex3d( scale , 0 , 0 ); // End
		glColor3f( 0 , 1 , 0 ); // G
		glVertex3d( 0     , 0 , 0 ); // Bgn
		glVertex3d( 0 , scale , 0 ); // End
		glColor3f( 99/255.0 , 133/255.0 , 255/255.0 ); // B-ish
		glVertex3d( 0 , 0 , 0 ); // Bgn 
		glVertex3d( 0 , 0 , scale ); // End
	glEnd();
	
	//  Label axes
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	glRasterPos3d( scale , 0 , 0 ); // Do the next raster operation at the window position corresponding to 3D coords
	Print( "X" );
	glRasterPos3d( 0 , scale , 0 );
	Print( "Y" );
	glRasterPos3d( 0 , 0 , scale );
	Print( "Z" );
}

void draw_grid_org_XY( float gridSize , uint xPlusMinus , uint yPlusMinus , 
					   float lineThic , vec3e color ){
	// Draw a square grid centered at the origin, extending 'xPlusMinus' units in X and 'yPlusMinus' units in Y
	
	float xMin = - gridSize * xPlusMinus , 
		  xMax =   gridSize * xPlusMinus ,
		  yMin = - gridSize * yPlusMinus , 
		  yMax =   gridSize * yPlusMinus ;
	
	glLineWidth( lineThic );
	glClr3e( color ); 
	
	glBegin( GL_LINES );
	
		// 1. Draw the axis , X
		glVertex3d( 0 , yMin , 0 ); // Bgn
		glVertex3d( 0 , yMax , 0 ); // End
		// 2. Draw the axis , Y
		glVertex3d( xMin , 0 , 0 ); // Bgn
		glVertex3d( xMax , 0 , 0 ); // End
		
		// 3. Draw the grid , X
		for( uint i = 0 ; i < xPlusMinus ; i++ ){
			// Plus
			glVertex3d(  gridSize * i , yMin , 0 ); // Bgn
			glVertex3d(  gridSize * i , yMax , 0 ); // End
			// Minus
			glVertex3d( -gridSize * i , yMin , 0 ); // Bgn
			glVertex3d( -gridSize * i , yMax , 0 ); // End
		}
		
		// 3. Draw the grid , Y
		for( uint i = 0 ; i < yPlusMinus ; i++ ){
			// Plus
			glVertex3d( xMin ,  gridSize * i , 0 ); // Bgn
			glVertex3d( xMax ,  gridSize * i , 0 ); // End
			// Minus
			glVertex3d( xMin , -gridSize * i , 0 ); // Bgn
			glVertex3d( xMax , -gridSize * i , 0 ); // End
		}
		
	glEnd();
}

void Vertex_sphr( float th , float ph ){
	// Draw vertex in polar coordinates
	// Author: Willem A. (Vlakkies) Schreüder  
	// glColor3f( Cos( th )*Cos( th ) , Sin(ph)*Sin(ph) , Sin(th)*Sin(th));
	glVertex3d( Sin( th ) * Cos( ph ) , 
				Sin( ph ) , 
				Cos( th ) * Cos( ph ) );
}

void sphere2( float x , float y , float z , float r ){
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

void cube( float x , float y , float z ,
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

void draw_cylinder( const vec3e& origin  , typeF length , typeF radius , uint facets ,
					const vec3e& fillClr , float shiny ){
	// Draw a cylinder of 'length' and 'radius' with the center of one end at 'origin', with an 'axisDir'
	// NEW VERSION: Has normals for lighting , Calculates less circles		
	
	glMaterialf(  GL_FRONT , GL_SHININESS , shiny      ); // Set the material shininess
	glMaterialfv( GL_FRONT , GL_SPECULAR  , MATL_WHITE ); // Set the color of the specular reflection
	glMaterialfv( GL_FRONT , GL_EMISSION  , MATL_BLACK );
	
	// 1. Get cicle coordinates
	matXe baseCirc = circ_space( radius * 2.0f , facets , vec2e{ origin[0] , origin[1] } );
	matXe btmCirc  = pts_XY_at_Z( baseCirc , origin[2] );
	matXe topCirc  = pts_XY_at_Z( baseCirc , origin[2] + length );
	//~ vec3e currNorm{0,0,0};
	vec3e currPnt{0,0,0};
	
	// 1.5. Set color
	glClr3e( fillClr );
	
	// 2. Paint barrel
	glBegin( GL_QUAD_STRIP );
	uint len = baseCirc.rows();
	uint i = 0;
	for( i = 0 ; i < len ; i++ ){
		currPnt = btmCirc.row(i);
		//~ currNorm = currPnt - origin;
		glNrm3e( currPnt - origin );
		glVec3e( btmCirc.row(i) );
		glVec3e( topCirc.row(i) );
	}
	currPnt = btmCirc.row(0);
	//~ currNorm = currPnt - origin;
	glNrm3e( currPnt - origin );
	glVec3e( btmCirc.row(0) );
	glVec3e( topCirc.row(0) );
	glEnd();

	// 3. Paint top
	glBegin( GL_TRIANGLE_FAN );
	glNormal( vec3e{ 0 , 0 ,  1 } );
	for( i = 0 ; i < len ; i++ ){  glVec3e( topCirc.row(i) );  }
	glEnd();
	
	// 4. Paint bottom
	glBegin( GL_TRIANGLE_FAN );
	glNormal( vec3e{ 0 , 0 , -1 } );
	for( i = 0 ; i < len ; i++ ){  glVec3e( btmCirc.row(i) );  }
	glEnd();
}

void draw_cylinder( const vec3e& origin  , typeF length , typeF radius , uint facets ,
					const vec3e& fillClr , const vec3e& lineClr ){
	// Draw a cylinder of 'length' and 'radius' with the center of one end at 'origin', with an 'axisDir'
	// OLD VERSION: No normals, calculates excessive circles, Has outlines
	
	// 0. Init
	float radMargin = 1.0 + 2/200.0;
	float lineWidth = length / 100.0;
	glLineWidth( 2.0 );
	
	// 1. Get cicle coordinates
	matXe baseCirc = circ_space( radius * 2.0             , facets , vec2e{ origin[0] , origin[1] } );
	matXe lineCirc = circ_space( radius * 2.0 * radMargin , facets , vec2e{ origin[0] , origin[1] } );
	// 2. Get bottom circle
	matXe btmCirc   = pts_XY_at_Z( baseCirc , origin[2] );
	matXe btmLinOut = pts_XY_at_Z( lineCirc , origin[2] );
	matXe btmLineIn = pts_XY_at_Z( lineCirc , origin[2] + lineWidth );
	// 3. Get top circle
	matXe topCirc   = pts_XY_at_Z( baseCirc , origin[2] + length );
	matXe topLinOut = pts_XY_at_Z( lineCirc , origin[2] + length);
	matXe topLineIn = pts_XY_at_Z( lineCirc , origin[2] + length - lineWidth );
	// 3.5. Set color
	glClr3e( fillClr );
	// 4. Paint barrel
	glBegin( GL_QUAD_STRIP );
	uint len = baseCirc.size();
	uint i = 0;
	for( uint i = 0 ; i < len ; i++ ){
		glVec3e( btmCirc.row(i) );
		glVec3e( topCirc.row(i) );
	}
	glVec3e( btmCirc.row(0) );
	glVec3e( topCirc.row(0) );
	glEnd();
	// 5. Paint top
	glBegin( GL_TRIANGLE_FAN );
	for( i = 0 ; i < len - 1 ; i++ ){  glVec3e( topCirc.row(i) );  }
	glEnd();
	// 6. Paint bottom
	glBegin( GL_TRIANGLE_FAN );
	for( i = 0 ; i < len - 1 ; i++ ){  glVec3e( btmCirc.row(i) );  }
	glEnd();
	// 7. Set line color
	glClr3e( lineClr );
	
	vec3e last;
	// 8. Paint top border
	glBegin( GL_LINES );
	last = topLineIn.row(0);
	for( uint i = 1 ; i < len ; i++ ){
		glVec3e( last );
		glVec3e( topLineIn.row(i) );
		last = topLineIn.row(i);
	}
	glVec3e( last );
	glVec3e( topLineIn.row(0) );
	glEnd();
	// 9. Paint bottom border
	glBegin( GL_LINES );
	last = btmLineIn.row(0);
	for( uint i = 1 ; i < len ; i++ ){
		glVec3e( last );
		glVec3e( btmLineIn.row(i) );
		last = btmLineIn.row(i);
	}
	glVec3e( last );
	glVec3e( btmLineIn.row(0) );
	glEnd();
}

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

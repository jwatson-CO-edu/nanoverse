/***********  
OGL_utils.c
James Watson , 2018 September , Although many of these functions are by others
Convenience functions for OpenGL

Template Version: 2017-09-23
***********/

#include "OGL_utils.h"

// === Classes and Structs =================================================================================================================



// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

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
					   float lineThic ){
	// Draw a square grid centered at the origin, extending 'xPlusMinus' units in X and 'yPlusMinus' units in Y
	
	float xMin = - gridSize * xPlusMinus , 
		  xMax =   gridSize * xPlusMinus ,
		  yMin = - gridSize * yPlusMinus , 
		  yMax =   gridSize * yPlusMinus ;
	
	glLineWidth( lineThic );
	glColor3f( 1 , 1 , 1 ); 
	
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
	glVertex3d( Sinf( th ) * Cosf( ph ) , 
				Sinf( ph ) , 
				Cosf( th ) * Cosf( ph ) );
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

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

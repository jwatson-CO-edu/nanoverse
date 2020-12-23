/***********  
TEMPLATE.cpp
James Watson , YYYY-MM
A_ONE_LINE_DESCRIPTION_OF_THE_FILE

Template Version: 2020-12
***********/

#include "OGL_utils.hpp"

/***** Utility Functions *************************************************************************/

void glVec3e( const vec3e& v ){  glVertex3f( v[0] , v[1] , v[2] );  } // Set a vertex with an Eigen vector
void glNrm3e( const vec3e& n ){  glNormal3f( n[0] , n[1] , n[2] );  } // Set a normal with an Eigen vector
void glClr3e( const vec3e& c ){  glColor3f(  c[0] , c[1] , c[2] );  } // Set the color with an Eigen vector

/*** Errors ***/

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


/*** Text ***/
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


/*** Simple Graphics ***/

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


/***** CLASNAME_! ********************************************************************************/
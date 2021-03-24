/***********  
OGL_utils.cpp
James Watson , 2021-01
Common OGL templates and functions

Template Version: 2020-12
***********/

#include "OGL_utils.hpp"

/***** Utility Functions *************************************************************************/

/*** Window & Context ***/
int init_GLUT( 
	int argc, char **argv , // --- Terminal args
	OGL_ContextConfig& params , // Window param structure
	void (*displayCB)() // ------- Display callback
){
	// Start FreeGLUT with the specified parameters
	glutInit( &argc, argv ); // ---------------------------------------- 1. Start the OGL utilities
	glutInitDisplayMode( params.displayMode ); // ---------------------- 2. Set the display mode
	glutInitWindowSize( params.screenDims[0], params.screenDims[1] ); // 3. Set initial window size
	glutInitWindowPosition( params.screenPosInit[0] , // --------------- 4. Set initial win position
							params.screenPosInit[1] ); 
	params.winHandle = glutCreateWindow( argv[0] ); // ----------------- 5. Create OGL window, get handle
	glutDisplayFunc( displayCB ); // ----------------------------------- 6. Set window display callback func
	return 0;
}

int set_redraw_functions(
	void (*reshapeCB)( int , int ) , // - Window reshape callback
	void (*timerCB)( int ) , // Periodic redraw function (Only set ONE)
	int  refreshPeriod_ms , // ----- Period to run `timerCB`
	void (*idleCB)() // - Idle redraw function (Only set ONE)
){
	// Set the reshape function
	if( reshapeCB )
		glutReshapeFunc( reshapeCB ); // Set window reshape callback func
	else
		glutReshapeFunc( dflt_reshapeCB );

	// Set EITHER a timer function or an idle function
	if( timerCB )
		glutTimerFunc( refreshPeriod_ms, timerCB, refreshPeriod_ms); // redraw only every given millisec
	else if( idleCB )
		glutIdleFunc( idleCB ); // redraw when idle
	else
		glutIdleFunc( dflt_idleCB ); // redraw when idle
}




void init_OGL( OGL_ContextConfig& params ){
	// Initialize OpenGL according to the performance preference
	glShadeModel( params.shadeModel ); // ----- Shader model: Smooth -or- Flat
	glPixelStorei( GL_UNPACK_ALIGNMENT, 8 ); // 8-byte pixel alignment for 64bit machine
	glHint( GL_PERSPECTIVE_CORRECTION_HINT, params.qualityHints );
	glEnable( GL_DEPTH_TEST ); // Enable depth test
	glDepthFunc( GL_LESS ); // // Accept fragment if it closer to the camera than the former one
    glEnable( GL_LIGHTING   );
    glEnable( GL_TEXTURE_2D );
    glEnable( GL_CULL_FACE  );
	// track material ambient and diffuse from surface color
	// call it before glEnable(GL_COLOR_MATERIAL)
    glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable( GL_COLOR_MATERIAL );
	glClearColor( // background color
		params.BGcolor[0], params.BGcolor[1], params.BGcolor[2], params.BGcolor[3] 
	);                   
	glClearStencil( 0 );  // clear stencil buffer
    glClearDepth( 1.0f ); // 0 is near, 1 is far
    
}

void create_light_source( LightSourceConfig& liteSpec ){
	// Create a lightsource with the given specification
	glLightfv( liteSpec.name, GL_AMBIENT,  liteSpec.lightKa );
    glLightfv( liteSpec.name, GL_DIFFUSE,  liteSpec.lightKd );
    glLightfv( liteSpec.name, GL_SPECULAR, liteSpec.lightKs );

	// position the light
    if( liteSpec.isPositional ){
		glLightfv( liteSpec.name, GL_POSITION, liteSpec.position );
	}
    
    glEnable( liteSpec.name ); // MUST enable each light source after configuration
}

void set_light_number( LightSourceConfig& config, GLenum lightName ){
	// Convert the light name to an appropriate entry in the light bit vector
	config.name   = lightName;
	config.number = 1 << (lightName - 0x0400);
}

void default_light_source(){
	// Create a default light source
	// 1. Init the light params
	LightSourceConfig config;
	set_light_number( config, GL_LIGHT0 );
	config.isPositional = true;	// --------------- Is this light at a certain position?
	/** Position **/
		config.position[0] = 0.0;
		config.position[1] = 0.0;
		config.position[2] = 0.0;
		config.position[3] = 1.0;
	/** Ambient Light **/
		config.lightKa[0] = 0.0;
		config.lightKa[1] = 0.0;
		config.lightKa[2] = 0.0;
		config.lightKa[3] = 0.0;
	/** Diffuse Light **/
		config.lightKd[0] = 0.0;
		config.lightKd[1] = 0.0;
		config.lightKd[2] = 0.0;
		config.lightKd[3] = 0.0;
	/** Specular Light **/
		config.lightKs[0] = 0.0;
		config.lightKs[1] = 0.0;
		config.lightKs[2] = 0.0;
		config.lightKs[3] = 0.0;
		
	// 2. Set the light params
	create_light_source( config );
}

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

// Set a vertex with an Eigen vector
static inline void glVec3e( const vec3e& v ){  glVertex3f( v[0] , v[1] , v[2] );  } 
// Set a normal with an Eigen vector
static inline void glNrm3e( const vec3e& n ){  glNormal3f( n[0] , n[1] , n[2] );  } 
// Set the color with an Eigen vector
static inline void glClr3e( const vec3e& c ){  glColor3f(  c[0] , c[1] , c[2] );  } 

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

/*** Defaults ***/
void dflt_displayCB(){ /* NO-OP */ }
void dflt_reshapeCB( int a, int b ){  cout << "New Screen Shape: (" << a << ", " << b << ")" << endl;  }
void dflt_idleCB(){  glutPostRedisplay();  }


/***** CLASNAME_! ********************************************************************************/
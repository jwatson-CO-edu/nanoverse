// gcc -O3 -Wall 02_sphere.c -lglut -lGLU -lGL -lm -o sphere.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "OGL_Geo.h"


////////// SETTINGS ////////////////////////////////////////////////////////////////////////////////
const float _SCALE = 10.0; //- Scale Dimension


////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

typedef struct{
	// Holds geo info for a net of triangles
	uint    Nvrt; // Number of vertices
	uint    Ntri; // Number of triangles
	matx_Nx3f* vert; // Vertices: _________ Ntri X {x,y,z}
	matx_Nx3u* face; // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
	matx_Nx3f* norm; // Face Normals: _____ Ntri X {x,y,z}
	matx_Nx3u* adjc; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	matx_Nx3f* Xdir; // Local Face X: _____ Ntri X {x,y,z}
	matx_Nx3f* Ydir; // Local Face Y: _____ Ntri X {x,y,z}
}TriNet;

TriNet* alloc_net( uint Ntri_, uint Nvrt_ ){
	// Allocate mem for a `TriNet` with `N` faces
	TriNet* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
	rtnStruct->Nvrt = Nvrt_; // ------------------ Number of vertices
	rtnStruct->Ntri = Ntri_; // ------------------ Number of triangles
	rtnStruct->vert = matrix_new_Nx3f( Nvrt_ ); // Vertices: _________ Ntri X {x,y,z}
	rtnStruct->face = matrix_new_Nx3u( Ntri_ ); // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
	rtnStruct->norm = matrix_new_Nx3f( Ntri_ ); // Face Normals: _____ Ntri X {x,y,z}
	rtnStruct->adjc = matrix_new_Nx3u( Ntri_ ); // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	rtnStruct->Xdir = matrix_new_Nx3f( Ntri_ ); // Local Face X: _____ Ntri X {x,y,z}
	rtnStruct->Ydir = matrix_new_Nx3f( Ntri_ ); // Local Face Y: _____ Ntri X {x,y,z}
	return rtnStruct;
}

void delete_net( TriNet* net ){
	// Free mem for a `TriNet` with `N` faces
	matrix_del( net->vert, net->Nvrt );
	matrix_del( net->face, net->Ntri );
	matrix_del( net->norm, net->Ntri );
	matrix_del( net->adjc, net->Ntri );
	matrix_del( net->Xdir, net->Ntri );
	matrix_del( net->Ydir, net->Ntri );
	free( net );
}

void load_vec3f_row( matx_Nx3f* matx, size_t i, float x, float y, float z ){
	// Load an R^3 vector into row `i` of `matx`
	(*matx)[i][0] = x;
	(*matx)[i][1] = y;
	(*matx)[i][2] = z;
}

void populate_icos_vertices( matx_Nx3f* V, float radius ){
	float sqrt5 = (float) sqrt( 5.0 ); // ----------------------------------- Square root of 5
	float phi   = (float)( 1.0 + sqrt5 ) * 0.5; // ------------------------- The Golden Ratio
	float ratio = (float)sqrt( 10.0 + ( 2.0 * sqrt5 ) ) / ( 4.0 * phi ); // ratio of edge length to radius
	float a     = ( radius / ratio ) * 0.5;
	float b     = ( radius / ratio ) / ( 2.0f * phi );
	load_vec3f_row( V,0, 0,b,-a );
	// FIXME, START HERE: LOAD VERTICES AND FACES
}


////////// VIEW PROJECTION /////////////////////////////////////////////////////////////////////////
float w2h =  0.0f; // Aspect ratio
int   fov = 55; // -- Field of view (for perspective)

static void Project(){
	// Set projection
	// Adapted from code provided by Willem A. (Vlakkies) Schreüder  
	// NOTE: This function assumes that aspect rario will be computed by 'resize'
	
	//  Tell OpenGL we want to manipulate the projection matrix
	glMatrixMode( GL_PROJECTION );
	//  Undo previous transformations
	glLoadIdentity();
	
	gluPerspective( (double) fov , // -- Field of view angle, in degrees, in the y direction.
					(double) w2h , // -- Aspect ratio , the field of view in the x direction. Ratio of x (width) to y (height).
					(double) _SCALE/4.0 , //- Specifies the distance from the viewer to the near clipping plane (always positive).
					(double) 4.0*_SCALE ); // Specifies the distance from the viewer to the far clipping plane (always positive).
	
	// Switch back to manipulating the model matrix
	glMatrixMode( GL_MODELVIEW );
	// Undo previous transformations
	glLoadIdentity();
}

////////// GEOMETRY & CAMERA ///////////////////////////////////////////////////////////////////////
Camera3D cam = { {5.0f, 2.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };



////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////
int    th     =  60; // Azimuth of view angle
int    ph     = 220; // Elevation of view angle

void display(){
	// Display the scene
	// Adapted from code provided by Willem Schreuder
	
    vec3f sphClr = {0.0f,1.0f,0.0f};
    vec3f center = {0.0f,0.0f,0.0f};

	//  Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	// Set view 
	look( cam );

	
	draw_sphere( center, 3.0f, sphClr );

	// Display status
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	Print( "HELLO WORLD!" );

	//  Flush and swap
	glFlush();
	glutSwapBuffers();
}



////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////
double dim    =  40; // Dimension of orthogonal box

void reshape( int width , int height ){
	// GLUT calls this routine when the window is resized
	// Calc the aspect ratio: width to the height of the window
	w2h = ( height > 0 ) ? (float) width / height : 1;
	// Set the viewport to the entire window
	glViewport( 0 , 0 , width , height );
	// Set projection
	Project();
}

int main( int argc , char* argv[] ){
	
	// // Calc the initial system before the user changes it
	// lorenz_system_trace( &FUNCTRACE , SIGMA , RHO , BETA , 50 , 0.001 , &N_FUNC );
	
	//  Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	
	//  Request 500 x 500 pixel window
	glutInitWindowSize( 1000 , 750 );
	
	//  Create the window
	glutCreateWindow( "LOOK AT THIS GODDAMN SPHERE" );

    // NOTE: Set modes AFTER the window / graphics context has been created!
    //  Request double buffered, true color window 
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
    // Enable z-testing at the full ranger
	glEnable( GL_DEPTH_TEST );
	glDepthRange( 0.0f , 1.0f );
	
	//  Tell GLUT to call "display" when the scene should be drawn
	glutDisplayFunc( display );
	
	//  Tell GLUT to call "reshape" when the window is resized
	glutReshapeFunc( reshape );
	
	// //  Tell GLUT to call "special" when an arrow key is pressed
	// glutSpecialFunc( special );
	
	// //  Tell GLUT to call "key" when a key is pressed
	// glutKeyboardFunc( key );
	
	//  Pass control to GLUT so it can interact with the user
	glutMainLoop();
	
	// // Free memory
	// matrix_del_f( FUNCTRACE , N_FUNC );
	
	//  Return code
	return 0;
}
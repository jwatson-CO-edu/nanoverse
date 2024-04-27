// gcc -O3 -Wall 04_tetra-net-test.c -lglut -lGLU -lGL -lm -o tetra.out
// 2024-04-27, WARNING: THIS FILE HAS NOT BEEN UPDATED TO THE L2R ARGUMENT CONVENTION, NON-FUNCTIONAL!
// 2024-04-27, WARNING: THIS FILE HAS NOT BEEN UPDATED TO THE L2R NAMING   CONVENTION, NON-FUNCTIONAL!

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "PolyNet.h"


////////// SETTINGS ////////////////////////////////////////////////////////////////////////////////
const float _SCALE = 10.0; //- Scale Dimension

///// Tetrahedron /////////////////////////////////////////////////////////

void populate_tetra_vertices_and_faces( matx_Nx3f* V, matx_Nx3u* F, float radius ){
	// Load geometry for an icosahedron onto matrices `V` and `F` 
	/// Calc req'd constants ///
    float a = radius;
    float b = radius / sqrtf( 2.0f );
	/// Load Vertices ///
	// Assume `V` already allocated for *4* vertices
    load_3f_to_row( V, 0,   a   , 0.0f,-b );
    load_3f_to_row( V, 1,  -a   , 0.0f,-b );
    load_3f_to_row( V, 2,   0.0f, a   , b );
    load_3f_to_row( V, 3,   0.0f,-a   , b );
	/// Load Faces ///
	// Assume `F` already allocated for *4* faces
	load_3u_to_row( F, 0,  0,1,2 );
	load_3u_to_row( F, 1,  0,2,3 );
	load_3u_to_row( F, 2,  0,3,1 );
	load_3u_to_row( F, 3,  1,3,2 );
}

TriNet* create_tetra_mesh_only( float radius ){
	// Create an regular icosahedron (*without* unfolded net data)
	/// Allocate ///
	TriNet* tetraNet = alloc_net( 4, 4 );
	/// Vertices and Faces ///
	populate_tetra_vertices_and_faces( tetraNet->vert, tetraNet->face, radius );
	/// Normals ///
	N_from_VF( tetraNet->Ntri, tetraNet->vert, tetraNet->face, tetraNet->norm );
	/// Return ///
	return tetraNet;
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
TriNet*  tetra;



////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////

void display(){
	// Display the scene
	// Adapted from code provided by Willem Schreuder
	
    vec3f netClr = {1.0f,1.0f,1.0f};
    vec3f conClr = {0.0f,1.0f,0.0f};
    // vec3f center = {0.0f,0.0f,0.0f};
    // vec3f sphClr = {0.0, 14.0f/255.0f, 214.0f/255.0f};

	//  Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	// Set view 
	look( cam );

	// draw_sphere( center, 1.5f, sphClr );
	draw_net_wireframe( tetra, netClr );
	draw_net_connectivity( tetra, conClr );

	// Display status
	glColor3f( 249/255.0 , 255/255.0 , 99/255.0 ); // Text Yellow
	glWindowPos2i( 5 , 5 ); // Next raster operation relative to lower lefthand corner of the window
	Print( "Icosahedron Net Connectivity" );

	//  Flush and swap
	glFlush();
	glutSwapBuffers();
}



////////// WINDOW STATE ////////////////////////////////////////////////////////////////////////////

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
	
	tetra = create_tetra_mesh_only( 2.00 );
	// adjacency_from_VF( icos->Ntri, 0.01, icos->vert, icos->face, icos->adjc );
	populate_net_connectivity_and_facet_frames( tetra, 0.01 );

	p_net_faces_outward_convex( tetra->Ntri, tetra->Nvrt, tetra->vert, tetra->face, tetra->norm );
	
	//  Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	//  Request 500 x 500 pixel window
	glutInitWindowSize( 1000 , 750 );
	
	//  Create the window
	glutCreateWindow( "LOOK AT THIS GODDAMN PLANET" );

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
	delete_net( tetra );
	
	//  Return code
	return 0;
}
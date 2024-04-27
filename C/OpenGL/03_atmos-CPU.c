// gcc -O3 -Wall 03_atmos-CPU.c -lglut -lGLU -lGL -lm -o atmos.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "PolyNet.h"


////////// SETTINGS ////////////////////////////////////////////////////////////////////////////////
const float _SCALE = 10.0; //- Scale Dimension






///// Triangular Cell /////////////////////////////////////////////////////

typedef struct{
	// Holds particle info for one triangular cell
	uint /*-*/ Nmax; // ---- Max number of particles that can occupy this cell
	uint /*-*/ insrtDex; //- Index for next insert
	vec2f /**/ accel; // --- Per-frame change in velocity
	float /**/ speedLim; //- Max speed of any particle
	vec2f /**/ v1; // ------ Vertex 1 in local frame
	vec2f /**/ v2; // ------ Vertex 2 in local frame
	uint /*-*/ ID; // ------ Triangle index in the mesh associated with this cell
	vec3u /**/ neighbors; // Local connectivity
	matx_Nx4f* prtLocVel; // Local position and velocity of each particle
	uint* /**/ triDices; //- Cell membership of each particle
}TriCell;


TriCell* alloc_cell( uint prtclMax_ ){
	// Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
	TriCell* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
	rtnStruct->prtLocVel = matrix_new_Nx4f( prtclMax_ );
	rtnStruct->triDices  = malloc( sizeof( uint ) * prtclMax_ );
	return rtnStruct;
}


void delete_cell( TriCell* cell ){
	// Free mem for a `TriCell` 
	free( cell->prtLocVel );
	free( cell->triDices );
	free( cell );
}


void init_cell( TriCell* cell, uint Nadd, float dimLim, float speedLim_, uint id ){
	// Populate particles with zero velocity, Set speed limit
	uint actAdd = min_uint( cell->Nmax, Nadd );
	for( uint i = 0; i < Nadd; ++i ){
		load_4f_to_row( cell->prtLocVel, i, randf()*dimLim, randf()*dimLim, 0.0f, 0.0f );
	}
	if( actAdd < cell->Nmax ){
		for( uint i = actAdd; i < cell->Nmax; ++i ){
			load_4f_to_row( cell->prtLocVel, i, 0.0f, 0.0f, 0.0f, 0.0f );
		}
	}
	cell->insrtDex = actAdd;
	cell->speedLim = fabsf( speedLim_ );
	cell->ID /*-*/ = id;
}


void set_cell_bounds( TriCell* cell, vec3f* origin, vec3f* xDir, vec3f* yDir, vec3f* v1_3f, vec3f* v2_3f ){
	// Locate the vertices of the 2D cell
	vec3f v1delta;  sub_vec3f( v1_3f, origin, &v1delta );
	vec3f v2delta;  sub_vec3f( v2_3f, origin, &v2delta );
	cell->v1[0] = dot_vec3f( xDir, &v1delta );
	cell->v1[1] = dot_vec3f( yDir, &v1delta );
	cell->v2[0] = dot_vec3f( xDir, &v2delta );
	cell->v2[1] = dot_vec3f( yDir, &v2delta );
}


void advance_particles( TriCell* cell ){
	// Perform one tick of the simulation
	float pX  = 0.0f;
	float pY  = 0.0f;
	float vX  = 0.0f;
	float vY  = 0.0f;
	// 1. For every particle
	for( uint i = 0; i < cell->Nmax; ++i ){
		// 2. Load particle position
		pX = (*cell->prtLocVel)[i][0];
		pY = (*cell->prtLocVel)[i][1];
		// 3. If particle is valid and it belongs to this cell, then ...
		if( ((vX != 0.0f)||(vY != 0.0f)) && (cell->triDices[i] == cell->ID) ){
			// 4. Load particle velocity
			vX = (*cell->prtLocVel)[i][2];
			vY = (*cell->prtLocVel)[i][3];
			// 5. Accelerate
			vX += cell->accel[0];
			vY += cell->accel[1];
			// 6. Apply per-direction speed limit
			vX /= fmaxf(fabsf(vX)/cell->speedLim, 1.0);
			vY /= fmaxf(fabsf(vY)/cell->speedLim, 1.0);
			// 7. Move particle
			pX += vX;
			pY += vY;
			// 8. Store updated particle info
			load_4f_to_row( cell->prtLocVel, i, pX, pY, vX, vY );
		}
		
	}
}


// ///// Particle Atmosphere /////////////////////////////////////////////////

// typedef struct{
// 	// Holds net and cell data for a toy atmosphere with cells containing particles that move in a planar fashion
// 	uint /**/ Ncells; // Number of cells in this atmosphere
// 	TriNet*   net; // -- Geometric information for atmosphere
// 	TriCell** cells; //- Array of pointers to cells containing particles
// }Atmos;


// Atmos* alloc_atmos( float radius, uint Nvrt_, uint Ncells_, uint prtclMax_ ){
// 	// Allocate memory for a toy atmosphere
// 	Atmos* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
// 	rtnStruct->Ncells = Ncells_;
// 	rtnStruct->cells  = malloc( sizeof( TriCell* ) * Ncells_ ); 
// 	rtnStruct->net    = alloc_net( Ncells_, Nvrt_ );
// 	for( ubyte i = 0; i < Ncells_; ++i ){
// 		rtnStruct->cells[i] = alloc_cell( prtclMax_ );
// 	}
// 	return rtnStruct;
// }


// void delete_atmos( Atmos* atmos ){
// 	// Free memory for a toy atmosphere
// 	for( ubyte i = 0; i < atmos->Ncells; ++i ){
// 		delete_cell( atmos->cells[i] );
// 	}
// 	free( atmos->cells );
// 	delete_net( atmos->net );
// 	free( atmos );
// }



///// Icosahedron /////////////////////////////////////////////////////////

void populate_icos_vertices_and_faces( matx_Nx3f* V, matx_Nx3u* F, float radius ){
	// Load geometry for an icosahedron onto matrices `V` and `F` 
	/// Calc req'd constants ///
	float sqrt5 = (float) sqrt( 5.0 ); // ----------------------------------- Square root of 5
	float phi   = (float)( 1.0 + sqrt5 ) * 0.5; // ------------------------- The Golden Ratio
	float ratio = (float)sqrt( 10.0 + ( 2.0 * sqrt5 ) ) / ( 4.0 * phi ); // ratio of edge length to radius
	float a     = ( radius / ratio ) * 0.5;
	float b     = ( radius / ratio ) / ( 2.0f * phi );
	/// Load Vertices ///
	// Assume `V` already allocated for *12* vertices
	load_3f_to_row( V, 0,  0, b,-a ); 
	load_3f_to_row( V, 1,  b, a, 0 );
	load_3f_to_row( V, 2, -b, a, 0 );
	load_3f_to_row( V, 3,  0, b, a );
	load_3f_to_row( V, 4,  0,-b, a );
	load_3f_to_row( V, 5, -a, 0, b );
	load_3f_to_row( V, 6,  0,-b,-a );
	load_3f_to_row( V, 7,  a, 0,-b );
	load_3f_to_row( V, 8,  a, 0, b );
	load_3f_to_row( V, 9, -a, 0,-b );
	load_3f_to_row( V,10,  b,-a, 0 );
	load_3f_to_row( V,11, -b,-a, 0 );
	/// Load Faces ///
	// Assume `F` already allocated for *20* faces
	load_3u_to_row( F, 0,  2, 1, 0 );
	load_3u_to_row( F, 1,  1, 2, 3 );
	load_3u_to_row( F, 2,  5, 4, 3 );
	load_3u_to_row( F, 3,  4, 8, 3 );
	load_3u_to_row( F, 4,  7, 6, 0 );
	load_3u_to_row( F, 5,  6, 9, 0 );
	load_3u_to_row( F, 6, 11,10, 4 );
	load_3u_to_row( F, 7, 10,11, 6 );
	load_3u_to_row( F, 8,  9, 5, 2 );
	load_3u_to_row( F, 9,  5, 9,11 );
	load_3u_to_row( F,10,  8, 7, 1 );
	load_3u_to_row( F,11,  7, 8,10 );
	load_3u_to_row( F,12,  2, 5, 3 );
	load_3u_to_row( F,13,  8, 1, 3 );
	load_3u_to_row( F,14,  9, 2, 0 );
	load_3u_to_row( F,15,  1, 7, 0 );
	load_3u_to_row( F,16, 11, 9, 6 );
	load_3u_to_row( F,17,  7,10, 6 );
	load_3u_to_row( F,18,  5,11, 4 );
	load_3u_to_row( F,19, 10, 8, 4 );
}


TriNet* create_icos_mesh_only( float radius ){
	// Create an regular icosahedron (*without* unfolded net data)
	/// Allocate ///
	TriNet* icosNet = alloc_net( 20, 12 );
	/// Vertices and Faces ///
	populate_icos_vertices_and_faces( icosNet->vert, icosNet->face, radius );
	/// Normals ///
	N_from_VF( icosNet->Ntri, icosNet->vert, icosNet->face, icosNet->norm );
	/// Return ///
	return icosNet;
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
TriNet*  icos;



////////// RENDERING LOOP //////////////////////////////////////////////////////////////////////////

void display(){
	// Display the scene
	// Adapted from code provided by Willem Schreuder
	
    vec3f icsClr = {1.0f,1.0f,1.0f};
    vec3f conClr = {0.0f,1.0f,0.0f};
    vec3f center = {0.0f,0.0f,0.0f};
    vec3f sphClr = {0.0, 14.0f/255.0f, 214.0f/255.0f};

	//  Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	// Set view 
	look( cam );

	draw_sphere( center, 1.5f, sphClr );
	draw_net_wireframe( icos, icsClr );
	draw_net_connectivity( icos, conClr );

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
	
	icos = create_icos_mesh_only( 2.00 );
	// adjacency_from_VF( icos->Ntri, 0.01, icos->vert, icos->face, icos->adjc );
	populate_net_connectivity_and_facet_frames( icos, 0.01 );
	
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
	delete_net( icos );
	
	//  Return code
	return 0;
}
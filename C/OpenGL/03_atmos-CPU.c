// gcc -O3 -Wall 03_atmos-CPU.c -lglut -lGLU -lGL -lm -o atmos.out


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "OGL_Geo.h"


////////// SETTINGS ////////////////////////////////////////////////////////////////////////////////
const float _SCALE = 10.0; //- Scale Dimension


////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////

typedef struct{
	// Holds geo info for a net of triangles
	uint /*-*/ Nvrt; // Number of vertices
	uint /*-*/ Ntri; // Number of triangles
	matx_Nx3f* vert; // Vertices: _________ Ntri X {x,y,z}
	matx_Nx3u* face; // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
	matx_Nx3f* norm; // Face Normals: _____ Ntri X {x,y,z}, Normal is the "Zdir" of local coord system
	matx_Nx3u* adjc; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	matx_Nx3f* orgn; // Local Origin: _____ Ntri X {x,y,z}
	matx_Nx3f* xDir; // Local Face X: _____ Ntri X {x,y,z}
	matx_Nx3f* yDir; // Local Face Y: _____ Ntri X {x,y,z}
}TriNet;


TriNet* alloc_net( uint Ntri_, uint Nvrt_ ){
	// Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
	TriNet* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
	rtnStruct->Nvrt = Nvrt_; // ------------------ Number of vertices
	rtnStruct->Ntri = Ntri_; // ------------------ Number of triangles
	rtnStruct->vert = matrix_new_Nx3f( Nvrt_ ); // Vertices: _________ Ntri X {x,y,z}
	rtnStruct->face = matrix_new_Nx3u( Ntri_ ); // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
	rtnStruct->norm = matrix_new_Nx3f( Ntri_ ); // Face Normals: _____ Ntri X {x,y,z}
	rtnStruct->adjc = matrix_new_Nx3u( Ntri_ ); // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	rtnStruct->orgn = matrix_new_Nx3f( Ntri_ ); // Local Face Origin:  Ntri X {x,y,z}
	rtnStruct->xDir = matrix_new_Nx3f( Ntri_ ); // Local Face X: _____ Ntri X {x,y,z}
	rtnStruct->yDir = matrix_new_Nx3f( Ntri_ ); // Local Face Y: _____ Ntri X {x,y,z}
	return rtnStruct;
}


void delete_net( TriNet* net ){
	// Free mem for a `TriNet` 
	free( net->vert );
	free( net->face );
	free( net->norm );
	free( net->adjc );
	free( net->orgn );
	free( net->xDir );
	free( net->yDir );
	free( net );
}


void get_CCW_tri_norm( const vec3f* v0, const vec3f* v1, const vec3f* v2, vec3f* n ){
	// Find the normal vector `n` of a triangle defined by CCW vertices in R^3: {`v0`,`v1`,`v2`}
	vec3f r1;     sub( v1, v0, &r1 );
	vec3f r2;     sub( v2, v0, &r2 );
	vec3f xBasis; unit( &r1, &xBasis );
	vec3f vecB;   unit( &r2, &vecB );
	vec3f nBig;   cross( &xBasis, &vecB, &nBig );
	/*---------*/ unit( &nBig, n ); // This should already be normalized
}


void N_from_VF( uint Ntri_, const matx_Nx3f* V, const matx_Nx3u* F, matx_Nx3f* N ){
	// Calc all face normals (One per face)
	vec3f v0;
	vec3f v1;
	vec3f v2;
	vec3f n_i;
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		load_row_to_vec3f( V, (*F)[i][0], &v0 );
		load_row_to_vec3f( V, (*F)[i][1], &v1 );
		load_row_to_vec3f( V, (*F)[i][2], &v2 );
		get_CCW_tri_norm( &v0, &v1, &v2, &n_i );
		load_vec3f_to_row( N, i, &n_i );
	}
}


void adjacency_from_VF( uint Ntri_, float eps, const matx_Nx3f* V, const matx_Nx3u* F, matx_Nx3u* A ){
	// Find face adjacencies and store connectivity in `A`, O(n^2) in number of faces
	vec3u face_i = {0,0,0};
	vec3u face_j = {0,0,0};
	vec3f vert_a1;
	vec3f vert_a2;
	vec3f vert_b1;
	vec3f vert_b2;
	// bool  found = false;

	// 0. All potential edges begin as null, Use triangle's own index to show no neighbor is present
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		for( ubyte a = 0; a < 3; ++a ){
			(*A)[i][a] = i; 
		}
	}

	// 1. For each triangle `i`, load face indices, then ...
	for( uint i = 0 ; i < Ntri_ ; i++ ){
		load_row_to_vec3u( F, i, &face_i );
		// 3. For each segment `a` of triangle `i`, load endpoint verts, then ...
		for( uint a = 0; a < 3; a++ ){
			load_row_to_vec3f( V, face_i[a]      , &vert_a1 );
			load_row_to_vec3f( V, face_i[(a+1)%3], &vert_a2 );
			// 2. For each triangle `j`, load face indices, then ...
			for( uint j = 0 ; j < Ntri_ ; j++ ){
				if(i != j){
					load_row_to_vec3u( F, j, &face_j );
					// 4. For each segment `b` of triangle `j`, load endpoint verts, then ...
					for( uint b = 0; b < 3; b++ ){
						load_row_to_vec3f( V, face_j[b]      , &vert_b1 );
						load_row_to_vec3f( V, face_j[(b+1)%3], &vert_b2 );
						// 5. Triangles that share an edge will have their shared vertices listed in an order reverse of the other
						//    NOTE: We test distance instead of index because some meshes might not have shared vertices
						if( (diff( &vert_a1, &vert_b2 ) <= eps) && (diff( &vert_a2, &vert_b1 ) <= eps) ){
							(*A)[i][a] = j;
							break;
						}
					}
				}
			}
		}
	}
}


void populate_coord_sys_per_face( uint Ntri_, const matx_Nx3f* V, const matx_Nx3u* F, const matx_Nx3f* N, 
								  matx_Nx3f* orgMtx, matx_Nx3f* xMtx, matx_Nx3f* yMtx ){
	// Create a local reference frame for every face of the polyhedron
	vec3f v0;
	vec3f v1;
	vec3f n;
	vec3f xSide;
	vec3f xBasis;
	vec3f yBasis;
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		// Compute Frame //
		load_row_to_vec3f( V, (*F)[i][0], &v0 );
		load_row_to_vec3f( V, (*F)[i][1], &v1 );
		load_row_to_vec3f( N, i, &n );
		sub( &v1, &v0, &xSide );
		unit( &xSide, &xBasis );
		cross( &n, &xBasis, &yBasis );
		unit( &yBasis, &yBasis );
		// Store Frame //
		load_vec3f_to_row( orgMtx, i, &v0     );
		// load_vec3f_to_row( xMtx  , i, &xBasis );
		// load_vec3f_to_row( yMtx  , i, &yBasis );
	}
}


void populate_net_connectivity_and_facet_frames( TriNet* net, float eps ){
	// Get facet neighborhoods and local reference frames
	// Connectivity //
	adjacency_from_VF( net->Ntri, eps, net->vert, net->face, net->adjc );
	// Frames //
	populate_coord_sys_per_face( net->Ntri, net->vert, net->face, net->norm, net->orgn, net->xDir, net->yDir );
}


void draw_net_wireframe( TriNet* net, vec3f lineColor ){
	// Draw the net as a wireframe, NOTE: Only `vert` and `face` data req'd
	glClr3f( lineColor );
	glBegin( GL_LINES );
	for( uint i = 0; i < net->Ntri; ++i ){
		load_row_to_glVtx3f( net->vert, (*net->face)[i][0] );  load_row_to_glVtx3f( net->vert, (*net->face)[i][1] );
		load_row_to_glVtx3f( net->vert, (*net->face)[i][1] );  load_row_to_glVtx3f( net->vert, (*net->face)[i][2] );
		load_row_to_glVtx3f( net->vert, (*net->face)[i][2] );  load_row_to_glVtx3f( net->vert, (*net->face)[i][1] );
	}
	glEnd();
}

void draw_net_connectivity( TriNet* net, vec3f lineColor ){
	// Draw the net as a wireframe, NOTE: Only `vert` and `face` data req'd
	vec3f v0;
	vec3f v1;
	vec3f v2;
	vec3f triCntr;
	vec3f segCntr;
	vec3u neighbors = {0,0,0};
	glClr3f( lineColor );
	glBegin( GL_LINES );
	// 1. For each face
	for( uint i = 0; i < net->Ntri; ++i ){
		// 2. Calc center
		load_row_to_vec3f( net->vert, (*net->face)[i][0], &v0 );
		load_row_to_vec3f( net->vert, (*net->face)[i][1], &v1 );
		load_row_to_vec3f( net->vert, (*net->face)[i][2], &v2 );
		tri_center( &v0, &v1, &v2, &triCntr );
		// 3. Load neighborhood
		load_row_to_vec3u( net->adjc, i, &neighbors );
		// 4. Test each possible connection. If it exists, then draw a segment from center of face to center of shared edge
		if( neighbors[0] != i ){
			seg_center( &v0, &v1, &segCntr );
			glVtx3f( triCntr );  glVtx3f( segCntr );  
		}
		if( neighbors[1] != i ){
			seg_center( &v1, &v2, &segCntr );
			glVtx3f( triCntr );  glVtx3f( segCntr );  
		}
		if( neighbors[2] != i ){
			seg_center( &v2, &v0, &segCntr );
			glVtx3f( triCntr );  glVtx3f( segCntr );  
		}
	}
	glEnd();
}



///// Cloud Particle //////////////////////////////////////////////////////

typedef struct{
	// Holds particle info for one triangular cell
	vec2f     postn;
	vec2f     veloc;
	Particle* next;
}Particle;

void free_particle_LL( Particle* head_ ){
	// Free a linked list of particles
}

///// Triangular Cell /////////////////////////////////////////////////////

typedef struct{
	// Holds particle info for one triangular cell
	vec2f     accelrtn; // Per-frame change in velocity
	vec2f     v0; // ----- Vertex 0 in local frame
	vec2f     v1; // ----- Vertex 1 in local frame
	vec2f     v3; // ----- Vertex 2 in local frame
	Particle* head;
	Particle* tail;
	Particle* send;
	float     speedLim;
}TriCell;


TriCell* alloc_cell( uint prtclMax_ ){
	// Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
	TriCell* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
	rtnStruct->head = NULL;
	rtnStruct->tail = NULL;
	return rtnStruct;
}

void delete_cell( TriCell* cell ){
	// Free mem for a `TriCell` 
	Particle* currPart = NULL;
	
	free( cell );
}

void push_particle( TriCell* cell, vec2f postn_, vec2f veloc_ ){
	Particle* nuPrtcl = malloc( sizeof( *nuPrtcl ) ); 
	nuPrtcl->postn[0] = postn_[0];
	nuPrtcl->postn[1] = postn_[1];
	nuPrtcl->veloc[0] = veloc_[0];
	nuPrtcl->veloc[1] = veloc_[1];
	nuPrtcl->next     = NULL;
	cell->tail->next  = nuPrtcl;
	cell->tail /*--*/ = nuPrtcl;
}


void init_cell( TriCell* cell, uint Nadd, float dimLim, float speedLim_ ){
	// Populate particles with zero velocity, Set speed limit
	uint actAdd = min_uint( cell->prtclMax, Nadd );
	for( uint i = 0; i < Nadd; ++i ){
		load_2f_to_row( cell->position, i, randf()*dimLim, randf()*dimLim );
		load_2f_to_row( cell->velocity, i, 0.0f          , 0.0f           );
	}
	cell->N_prtcls = actAdd;
	cell->speedLim = fabsf( speedLim_ );
}

void advance_particles( TriCell* cell ){
	// Perform one tick of the simulation
}


///// Particle Atmosphere /////////////////////////////////////////////////

typedef struct{
	// Holds net and cell data for a toy atmosphere with cells containing particles that move in a planar fashion
	uint /**/ Ncells; // Number of cells in this atmosphere
	TriNet*   net; // -- Geometric information for atmosphere
	TriCell** cells; //- Array of pointers to cells containing particles
}Atmos;


Atmos* alloc_atmos( float radius, uint Nvrt_, uint Ncells_, uint prtclMax_ ){
	// Allocate memory for a toy atmosphere
	Atmos* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
	rtnStruct->Ncells = Ncells_;
	rtnStruct->cells  = malloc( sizeof( TriCell* ) * Ncells_ ); 
	rtnStruct->net    = alloc_net( Ncells_, Nvrt_ );
	for( ubyte i = 0; i < Ncells_; ++i ){
		rtnStruct->cells[i] = alloc_cell( prtclMax_ );
	}
	return rtnStruct;
}


void delete_atmos( Atmos* atmos ){
	// Free memory for a toy atmosphere
	for( ubyte i = 0; i < atmos->Ncells; ++i ){
		delete_cell( atmos->cells[i] );
	}
	free( atmos->cells );
	delete_net( atmos->net );
	free( atmos );
}



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
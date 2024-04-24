// gcc -O3 -Wall 03_atmos-CPU.c -lglut -lGLU -lGL -lm -o atmos.out


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
	matx_Nx3f* norm; // Face Normals: _____ Ntri X {x,y,z}, Normal is the "Zdir" of local coord system
	matx_Nx3u* adjc; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	matx_Nx3f* orgn; // Local Origin: _____ Ntri X {x,y,z}
	matx_Nx3f* Xdir; // Local Face X: _____ Ntri X {x,y,z}
	matx_Nx3f* Ydir; // Local Face Y: _____ Ntri X {x,y,z}
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
	rtnStruct->Xdir = matrix_new_Nx3f( Ntri_ ); // Local Face X: _____ Ntri X {x,y,z}
	rtnStruct->Ydir = matrix_new_Nx3f( Ntri_ ); // Local Face Y: _____ Ntri X {x,y,z}
	return rtnStruct;
}

void delete_net( TriNet* net ){
	// Free mem for a `TriNet` 
	free( net->vert );
	free( net->face );
	free( net->norm );
	free( net->adjc );
	free( net->Xdir );
	free( net->Ydir );
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

	// 0. All potential edges begin as null, Use triangle's own index to show no neighbor is present
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		for( uint j = 0 ; j < Ntri_ ; ++j ){
			for( ubyte a = 0; a < 3; ++a ){
				for( ubyte b = 0; b < 3; ++b ){
					(*A)[i][a] = i; 
					(*A)[j][b] = j;
				}
			}
		}
	}
	// 1. For each triangle `i`, load face indices, then ...
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		load_row_to_vec3u( F, i, &face_i );
		// 2. For each triangle `j`, load face indices, then ...
		for( uint j = 0 ; j < Ntri_ ; ++j ){
			load_row_to_vec3u( F, j, &face_j );
			// 3. For each segment `a` of triangle `i`, load endpoint verts, then ...
			for( ubyte a = 0; a < 3; ++a ){
				load_row_to_vec3f( V, face_i[a]      , &vert_a1 );
				load_row_to_vec3f( V, face_i[(a+1)%3], &vert_a2 );
				// 4. For each segment `b` of triangle `j`, load endpoint verts, then ...
				for( ubyte b = 0; b < 3; ++b ){
					load_row_to_vec3f( V, face_j[b]      , &vert_b1 );
					load_row_to_vec3f( V, face_j[(b+1)%3], &vert_b2 );
					// 5. Triangles that share an edge will have their shared vertices listed in an order reverse of the other
					//    NOTE: We test distance instead of index because some meshes might not have shared vertices
					if( (i != j) && (diff( &vert_a1, &vert_b2 ) <= eps) && (diff( &vert_a2, &vert_b1 ) <= eps) ){
						(*A)[i][a] = j;
						(*A)[j][b] = i;
					}
				}
			}
		}
	}
}

void populate_coord_sys_per_face( const matx_Nx3f* V, const matx_Nx3u* F, const matx_Nx3f* N, 
								  matx_Nx3f* orgMtx, matx_Nx3f* xMtx, matx_Nx3f* yMtx ){
	// Create a local reference frame for every face of the polyhedron

	// FIXME, START HERE: THIS IS JUST FETCHES AND CROSS PRODUCTS AND NORMALIZATION
	
}

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
	glClr3f( lineColor );
	glBegin( GL_LINES );
	vec3f v0;
	vec3f v1;
	vec3f v2;
	vec3f triCntr;
	vec3f segCntr;
	vec3u neighbors = {0,0,0};
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
int    th     =  60; // Azimuth of view angle
int    ph     = 220; // Elevation of view angle

void display(){
	// Display the scene
	// Adapted from code provided by Willem Schreuder
	
    vec3f icsClr = {0.0f,1.0f,0.0f};
    // vec3f center = {0.0f,0.0f,0.0f};

	//  Clear the image
	glClearDepth( 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	//  Reset previous transforms to the identity matrix
	glLoadIdentity();
	
	// Set view 
	look( cam );

	// draw_sphere( center, 3.0f, sphClr );
	// draw_net_wireframe( icos, icsClr );
	draw_net_connectivity( icos, icsClr );

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
	
	icos = create_icos_mesh_only( 2.0 );
	adjacency_from_VF( icos->Ntri, 0.01, icos->vert, icos->face, icos->adjc );
	
	//  Initialize GLUT and process user parameters
	glutInit( &argc , argv );
	
	//  Request 500 x 500 pixel window
	glutInitWindowSize( 1000 , 750 );
	
	//  Create the window
	glutCreateWindow( "LOOK AT THIS GODDAMN NETWORK" );

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
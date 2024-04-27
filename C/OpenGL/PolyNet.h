#ifndef POLYNET_H // This pattern is to prevent symbols to be loaded multiple times
#define POLYNET_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "OGL_Geo.h"

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
	vec3f r1;     sub_vec3f( v1, v0, &r1 );
	vec3f r2;     sub_vec3f( v2, v0, &r2 );
	vec3f xBasis; unit_vec3f( &r1, &xBasis );
	vec3f vecB;   unit_vec3f( &r2, &vecB );
	vec3f nBig;   cross_vec3f( &xBasis, &vecB, &nBig );
	printf( "Area: %f, ", norm_vec3f( &nBig )/2.0f );
	/*---------*/ unit_vec3f( &nBig, n ); // This should already be normalized
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
	vec3f vert_i1;
	vec3f vert_i2;
	vec3f vert_j1;
	vec3f vert_j2;
	uint  totMatch = 0;
	uint  totCompr = 0;
	bool  found    = false;

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
			load_row_to_vec3f( V, face_i[a]      , &vert_i1 );
			load_row_to_vec3f( V, face_i[(a+1)%3], &vert_i2 );
			// 2. For each triangle `j`, load face indices, then ...
			found = false;
			for( uint j = 0 ; j < Ntri_ ; j++ ){
				if(i != j){
					load_row_to_vec3u( F, j, &face_j );
					// 4. For each segment `b` of triangle `j`, load endpoint verts, then ...
					for( uint b = 0; b < 3; b++ ){
						load_row_to_vec3f( V, face_j[b]      , &vert_j1 );
						load_row_to_vec3f( V, face_j[(b+1)%3], &vert_j2 );
						// 5. Triangles that share an edge will have their shared vertices listed in an order reverse of the other
						//    NOTE: We test distance instead of index because some meshes might not have shared vertices
						
						printf( "Compare [%u,%u] to [%u,%u]: ",i,a,j,(b+1)%3 );  
						print_vec3f( vert_i1 );  print_vec3f( vert_j2 );  nl();

						printf( "Compare [%u,%u] to [%u,%u]: ",i,(a+1)%3,j,b );  
						print_vec3f( vert_i2 );  print_vec3f( vert_j1 );  nl();  

						++totCompr;

						if( (diff_vec3f( &vert_i1, &vert_j2 ) <= eps) && (diff_vec3f( &vert_i2, &vert_j1 ) <= eps) ){
							printf( "! MATCH !\n\n" );
							++totMatch;
							(*A)[i][a] = j;
							found = true;
						}

						nl();

						if( found )  break;
					}
				}
				if( found )  break;
			}
		}
	}
	printf( "##### There were %i total matches! #####\n", totMatch );
	printf( "\tThere were %i comparisons made! \n", totCompr );
}

bool p_net_faces_outward_convex( uint Ntri_, uint Nvrt_, const matx_Nx3f* V, const matx_Nx3u* F, const matx_Nx3f* N ){
	// Return true if the faces of an assumed convex net all face outwards
	vec3f total = {0.0f,0.0f,0.0f};
	vec3f oprnd = {0.0f,0.0f,0.0f};
	vec3f centr = {0.0f,0.0f,0.0f};
	vec3f v0_i  = {0.0f,0.0f,0.0f};
	vec3f ray_i = {0.0f,0.0f,0.0f};
	vec3f nrm_i = {0.0f,0.0f,0.0f};
	float dTest = 0.0f;
	bool  allOutward = true;
	// Calc Centroid //
	for( uint i = 0 ; i < Nvrt_ ; ++i ){
		load_row_to_vec3f( V, i, &oprnd );
		add_vec3f( &oprnd, &total, &total );
	}
	div_vec3f( &total, (float) Nvrt_, &centr );
	// Check Each Normal //
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		load_row_to_vec3f( V, (*F)[i][0], &v0_i  );
		load_row_to_vec3f( N, i         , &nrm_i );
		sub_vec3f( &v0_i, &centr, &ray_i );
		dTest = dot_vec3f( &ray_i, &nrm_i );
		printf( "%f, ", dTest );
		if( dTest < 0.0f ){
			printf( "Face %i should be REVERSED!\n", i );
			allOutward = false;
		}
	}
	if( allOutward ){  printf( "ALL triangles OKAY!\n" );  }
	return allOutward;
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
		sub_vec3f( &v1, &v0, &xSide );
		unit_vec3f( &xSide, &xBasis );
		cross_vec3f( &n, &xBasis, &yBasis );
		unit_vec3f( &yBasis, &yBasis );
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

#endif
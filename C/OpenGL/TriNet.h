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
	matx_Nx3f* V; // Vertices: _________ Ntri X {x,y,z}
	matx_Nx3u* F; // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
	matx_Nx3f* N; // Face Normals: _____ Ntri X {x,y,z}, Normal is the "Zdir" of local coord system
	matx_Nx3u* A; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	// matx_Nx3f* orgn; // Local Origin: _____ Ntri X {x,y,z}
	// matx_Nx3f* xDir; // Local Face X: _____ Ntri X {x,y,z}
	// matx_Nx3f* yDir; // Local Face Y: _____ Ntri X {x,y,z}
}TriNet;


TriNet* alloc_net( uint Ntri_, uint Nvrt_ ){
	// Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
	TriNet* rtnStruct = malloc( sizeof( *rtnStruct ) ); 
	rtnStruct->Nvrt = Nvrt_; // ------------------ Number of vertices
	rtnStruct->Ntri = Ntri_; // ------------------ Number of triangles
	rtnStruct->V = matrix_new_Nx3f( Nvrt_ ); // Vertices: _________ Ntri X {x,y,z}
	rtnStruct->F = matrix_new_Nx3u( Ntri_ ); // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
	rtnStruct->N = matrix_new_Nx3f( Ntri_ ); // Face Normals: _____ Ntri X {x,y,z}
	rtnStruct->A = matrix_new_Nx3u( Ntri_ ); // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
	// rtnStruct->orgn = matrix_new_Nx3f( Ntri_ ); // Local Face Origin:  Ntri X {x,y,z}
	// rtnStruct->xDir = matrix_new_Nx3f( Ntri_ ); // Local Face X: _____ Ntri X {x,y,z}
	// rtnStruct->yDir = matrix_new_Nx3f( Ntri_ ); // Local Face Y: _____ Ntri X {x,y,z}
	return rtnStruct;
}


void delete_net( TriNet* net ){
	// Free mem for a `TriNet` 
	free( net->V );
	free( net->F );
	free( net->N );
	free( net->A );
	// free( net->orgn );
	// free( net->xDir );
	// free( net->yDir );
	free( net );
}


void get_CCW_tri_norm( const vec3f* v0, const vec3f* v1, const vec3f* v2, vec3f* n ){
	// Find the normal vector `n` of a triangle defined by CCW vertices in R^3: {`v0`,`v1`,`v2`}
	vec3f r1;    sub_vec3f( &r1, v1, v0 );
	vec3f r2;    sub_vec3f( &r2, v2, v0 );
	vec3f nBig;  cross_vec3f( &nBig, &r1, &r2 );
	/*-------*/  unit_vec3f( n, &nBig );
}


float get_tri_area( const vec3f* v0, const vec3f* v1, const vec3f* v2){
	// Find the area of a triangle defined by vertices in R^3: {`v0`,`v1`,`v2`}
	vec3f r1;    sub_vec3f( &r1, v1, v0 );
	vec3f r2;    sub_vec3f( &r2, v2, v0 );
	vec3f nBig;  cross_vec3f( &nBig, &r1, &r2 );
	return /**/  norm_vec3f( &nBig )/2.0f;
}


void N_from_VF( uint Ntri_, const matx_Nx3f* V, const matx_Nx3u* F, matx_Nx3f* N ){
	// Calc all F normals (One per F)
	vec3f v0;
	vec3f v1;
	vec3f v2;
	vec3f n_i;
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		load_vec3f_from_row( &v0, V, (*F)[i][0] );
		load_vec3f_from_row( &v1, V, (*F)[i][1] );
		load_vec3f_from_row( &v2, V, (*F)[i][2] );
		get_CCW_tri_norm( &v0, &v1, &v2, &n_i );
		load_row_from_vec3f( N, i, &n_i );
	}
}


void A_from_VF( uint Ntri_, float eps, const matx_Nx3f* V, const matx_Nx3u* F, matx_Nx3u* A ){
	// Find F adjacencies and store connectivity in `A`, O(n^2) in number of faces
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

	// 1. For each triangle `i`, load F indices, then ...
	for( uint i = 0 ; i < Ntri_ ; i++ ){
		load_vec3u_from_row( &face_i, F, i );
		// 3. For each segment `a` of triangle `i`, load endpoint verts, then ...
		for( uint a = 0; a < 3; a++ ){
			load_vec3f_from_row( &vert_i1, V, face_i[a]       );
			load_vec3f_from_row( &vert_i2, V, face_i[(a+1)%3] );
			// 2. For each triangle `j`, load F indices, then ...
			found = false;
			for( uint j = 0 ; j < Ntri_ ; j++ ){
				if(i != j){
					load_vec3u_from_row( &face_j, F, j );
					// 4. For each segment `b` of triangle `j`, load endpoint verts, then ...
					for( uint b = 0; b < 3; b++ ){
						load_vec3f_from_row( &vert_j1, V, face_j[b]       );
						load_vec3f_from_row( &vert_j2, V, face_j[(b+1)%3] );
						// 5. Triangles that share an edge will have their shared vertices listed in an order reverse of the other
						//    NOTE: We test distance instead of index because some meshes might not have shared vertices

						++totCompr;

						if( (diff_vec3f( &vert_i1, &vert_j2 ) <= eps) && (diff_vec3f( &vert_i2, &vert_j1 ) <= eps) ){
							// printf( "! MATCH !\n\n" );
							++totMatch;
							(*A)[i][a] = j;
							found = true;
						}

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
	// Return true if the faces of an assumed convex net all F outwards
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
		load_vec3f_from_row( &oprnd, V, i );
		add_vec3f( &total, &oprnd, &total );
	}
	div_vec3f( &centr, &total, (float) Nvrt_ );
	// Check Each Normal //
	for( uint i = 0 ; i < Ntri_ ; ++i ){
		load_vec3f_from_row( &v0_i, V, (*F)[i][0] );
		load_vec3f_from_row( &nrm_i, N, i         );
		sub_vec3f( &ray_i, &v0_i, &centr );
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


// void populate_coord_sys_per_face( uint Ntri_, const matx_Nx3f* V, const matx_Nx3u* F, const matx_Nx3f* N, 
// 								  matx_Nx3f* orgMtx, matx_Nx3f* xMtx, matx_Nx3f* yMtx ){
// 	// Create a local reference frame for every F of the polyhedron
// 	vec3f v0;
// 	vec3f v1;
// 	vec3f n;
// 	vec3f xSide;
// 	vec3f xBasis;
// 	vec3f yBasis;
// 	for( uint i = 0 ; i < Ntri_ ; ++i ){
// 		// Compute Frame //
// 		load_vec3f_from_row( V, (*F)[i][0], &v0 );
// 		load_vec3f_from_row( V, (*F)[i][1], &v1 );
// 		load_vec3f_from_row( N, i, &n );

// 		// Store Frame //
// 		load_row_from_vec3f( orgMtx, i, &v0     );
// 		// load_row_from_vec3f( xMtx  , i, &xBasis );
// 		// load_row_from_vec3f( yMtx  , i, &yBasis );
// 	}
// }


void populate_net_connectivity( TriNet* net, float eps ){
	// Get facet neighborhoods 
	A_from_VF( net->Ntri, eps, net->V, net->F, net->A );
}


void draw_net_wireframe( TriNet* net, vec3f lineColor ){
	// Draw the net as a wireframe, NOTE: Only `V` and `F` data req'd
	glClr3f( lineColor );
	glBegin( GL_LINES );
	for( uint i = 0; i < net->Ntri; ++i ){
		send_row_to_glVtx3f( net->V, (*net->F)[i][0] );  send_row_to_glVtx3f( net->V, (*net->F)[i][1] );
		send_row_to_glVtx3f( net->V, (*net->F)[i][1] );  send_row_to_glVtx3f( net->V, (*net->F)[i][2] );
		send_row_to_glVtx3f( net->V, (*net->F)[i][2] );  send_row_to_glVtx3f( net->V, (*net->F)[i][1] );
	}
	glEnd();
}


void draw_net_connectivity( TriNet* net, vec3f lineColor ){
	// Draw the net as a wireframe, NOTE: Only `V` and `F` data req'd
	vec3f v0;
	vec3f v1;
	vec3f v2;
	vec3f triCntr;
	vec3f segCntr;
	vec3u neighbors = {0,0,0};
	glClr3f( lineColor );
	glBegin( GL_LINES );
	// 1. For each F
	for( uint i = 0; i < net->Ntri; ++i ){
		// 2. Calc center
		load_vec3f_from_row( &v0, net->V, (*net->F)[i][0] );
		load_vec3f_from_row( &v1, net->V, (*net->F)[i][1] );
		load_vec3f_from_row( &v2, net->V, (*net->F)[i][2] );
		tri_center( &triCntr, &v0, &v1, &v2 );
		// 3. Load neighborhood
		load_vec3u_from_row( &neighbors, net->A, i );
		// 4. Test each possible connection. If it exists, then draw a segment from center of F to center of shared edge
		if( neighbors[0] != i ){
			seg_center( &segCntr, &v0, &v1 );
			glVtx3f( triCntr );  glVtx3f( segCntr );  
		}
		if( neighbors[1] != i ){
			seg_center( &segCntr, &v1, &v2 );
			glVtx3f( triCntr );  glVtx3f( segCntr );  
		}
		if( neighbors[2] != i ){
			seg_center( &segCntr, &v2, &v0 );
			glVtx3f( triCntr );  glVtx3f( segCntr );  
		}
	}
	glEnd();
}

#endif
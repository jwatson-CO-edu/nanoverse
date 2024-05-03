#ifndef TRINET_H // This pattern is to prevent symbols to be loaded multiple times
#define TRINET_H // from multiple imports

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "ZZ_Utils.h"


vec4f sub_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` - `v` = `r`, R^3
    vec4f rtnVec = {
        u.x - v.x,
        u.y - v.y,
        u.z - v.z,
        1.0
    };
    return rtnVec;
}


vec4f add_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` + `v` = `r`, R^3
    vec4f rtnVec = {
        u.x + v.x,
        u.y + v.y,
        u.z + v.z,
        1.0
    };
    return rtnVec;
}


float dot_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` * `v` = `r`, R^3
    return u.x * v.x + u.y * v.y + u.z * v.z;
}


float norm_vec4f( const vec4f vec ){  
    // Euclidean length of an R^3
    return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z); // Assume w = 1.0f
} 


float diff_vec4f( const vec4f u, const vec4f v ){  
    // Euclidean length of `u`-`v`
    return norm_vec4f( sub_vec4f( u, v ) );
} 


vec4f unit_vec4f( const vec4f vec ){
    // Calc the unit direction of `vec` and return it, R^3
    vec4f  rtnVec;
    float mag = norm_vec4f( vec );
    if( mag > 0.0f ){
        rtnVec.x = vec.x / mag;
        rtnVec.y = vec.y / mag;
        rtnVec.z = vec.z / mag;
    }else{
        rtnVec.x = 0.0f;
        rtnVec.y = 0.0f;
        rtnVec.z = 0.0f;
    }
    rtnVec.w = 1.0f;
    return rtnVec;
}

vec4f cross_vec4f( const vec4f u, const vec4f v ){
    // Calc `u` X `v` = `p`, R^3
    // Source: http://aleph0.clarku.edu/~djoyce/ma131/dotcross.pdf , pg. 3
    vec4f rtnVec = {
        u.y*v.z - u.z*v.y,
        u.z*v.x - u.x*v.z,
        u.x*v.y - u.y*v.x,
        1.0f
    };
    return rtnVec;
}


vec4f div_vec4f( const vec4f u, float d ){
    // Calc `u` * `f` = `r`, R^3
    vec4f rtnVec = {
        u.x / d,
        u.y / d,
        u.z / d,
        1.0f
    };
    return rtnVec;
}


vec4f get_CCW_tri_norm( const vec4f v0, const vec4f v1, const vec4f v2 ){
    // Find the normal vector `n` of a triangle defined by CCW vertices in R^3: {`v0`,`v1`,`v2`}
    return unit_vec4f( cross_vec4f(
        sub_vec4f( v1, v0 ),
        sub_vec4f( v2, v0 )
    ) );
}

vec4f tri_center( const vec4f v0, const vec4f v1, const vec4f v2 ){
    // Calc centroid of 3 R^3 points
    vec4f rtnVec = {
        (v0.x + v1.x + v2.x) / 3.0f,
        (v0.y + v1.y + v2.y) / 3.0f,
        (v0.z + v1.z + v2.z) / 3.0f,
        1.0f
    };
    return rtnVec;
}

////////// OPENGL HELPERS //////////////////////////////////////////////////////////////////////////

void glVtx4f( const vec4f v ){  glVertex3f( v.x , v.y , v.z );  } // Set vertex with a vector
void glNrm4f( const vec4f n ){  glNormal3f( n.x , n.x , n.x );  } // Set normal with a vector
void glClr4f( const vec4f c ){  glColor4f(  c.r , c.g , c.b, c.a );  } // Set color with a vector

////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////

typedef struct{
    // Holds geo info for a polyhedral net of triangles // WARNING: THERE SHOULD BE A NORMAL FOR EACH VERTEX
    uint /*-*/ Nvrt; // Number of vertices
    uint /*-*/ Ntri; // Number of triangles
    vec4f* V; // Vertices: _________ Ntri X {x,y,z}
    vec3u* F; // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    vec4f* N; // Face Normals: _____ Ntri X {x,y,z}, Normal is the "Zdir" of local coord system
    vec3u* A; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
}TriNet;


TriNet* alloc_net( uint Ntri_, uint Nvrt_ ){
    // Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
    TriNet* rtnStruct = (TriNet*) malloc( sizeof( TriNet ) ); 
    rtnStruct->Nvrt = Nvrt_; // ------------------ Number of vertices
    rtnStruct->Ntri = Ntri_; // ------------------ Number of triangles
    rtnStruct->V = (vec4f*) malloc( Nvrt_ * sizeof( vec4f ) ); // Vertices: _________ Nvrt X {x,y,z}
    rtnStruct->F = (vec3u*) malloc( Ntri_ * sizeof( vec3u ) ); // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    rtnStruct->N = (vec4f*) malloc( Nvrt_ * sizeof( vec4f ) ); // Face Normals: _____ Nvrt X {x,y,z}
    rtnStruct->A = (vec3u*) malloc( Ntri_ * sizeof( vec3u ) );// Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
    return rtnStruct;
}


void delete_net( TriNet* net ){
    // Free mem for a `TriNet` 
    free( net->V );
    free( net->F );
    free( net->N );
    free( net->A );
    free( net );
}


void N_from_VF( vec4f* N, /*<<*/ uint Ntri_, const vec4f* V, const vec3u* F ){
    // Calc all F normals (One per F)
    vec4f v0;
    vec4f v1;
    vec4f v2;
    vec4f n_i;
    for( uint i = 0 ; i < Ntri_ ; ++i ){
        v0 = V[ F[i].v0 ];
        v1 = V[ F[i].v1 ];
        v2 = V[ F[i].v2 ];
        n_i = get_CCW_tri_norm( v0, v1, v2 );
        N[ F[i].v0 ] = n_i;
        N[ F[i].v1 ] = n_i;
        N[ F[i].v2 ] = n_i;
    }
}

void set_uintArr3_from_vec3u( uint* arr, const vec3u vec ){
    arr[0] = vec.v0;
    arr[1] = vec.v1;
    arr[2] = vec.v2;
}

void A_from_VF( vec3u* A, /*<<*/ uint Ntri_, float eps, const vec4f* V, const vec3u* F ){
    // Find F adjacencies and store connectivity in `A`, O(n^2) in number of faces
    uint face_i[3] = {0,0,0};
    uint face_j[3] = {0,0,0};
    vec4f vert_i1;
    vec4f vert_i2;
    vec4f vert_j1;
    vec4f vert_j2;
    uint  totMatch = 0;
    uint  totCompr = 0;
    bool  found    = false;

    // 0. All potential edges begin as null, Use triangle's own index to show no neighbor is present
    for( uint i = 0 ; i < Ntri_ ; ++i ){
        A[i].f0 = i;
        A[i].f1 = i;
        A[i].f2 = i;
    }

    // 1. For each triangle `i`, load F indices, then ...
    for( uint i = 0 ; i < Ntri_ ; i++ ){
        set_uintArr3_from_vec3u( face_i, F[i] );
        // 3. For each segment `a` of triangle `i`, load endpoint verts, then ...
        for( uint a = 0; a < 3; a++ ){
            vert_i1 = V[ face_i[a      ] ];
            vert_i2 = V[ face_i[(a+1)%3] ];
            // 2. For each triangle `j`, load F indices, then ...
            found = false;
            for( uint j = 0 ; j < Ntri_ ; j++ ){
                if(i != j){
                    set_uintArr3_from_vec3u( face_j, F[j] );
                    // 4. For each segment `b` of triangle `j`, load endpoint verts, then ...
                    for( uint b = 0; b < 3; b++ ){
                        vert_j1 = V[ face_i[b      ] ];
                        vert_j2 = V[ face_i[(b+1)%3] ];
                        // 5. Triangles that share an edge will have their shared vertices listed in an order reverse of the other
                        //    NOTE: We test distance instead of index because some meshes might not have shared vertices
                        ++totCompr;
                        if( (diff_vec4f( vert_i1, vert_j2 ) <= eps) && (diff_vec4f( vert_i2, vert_j1 ) <= eps) ){
                            // printf( "! MATCH !\n\n" );
                            ++totMatch;
                            switch(a){
                                case 0:
                                    A[i].f0 = j;
                                    break;
                                case 1:
                                    A[i].f1 = j;
                                    break;
                                case 2:
                                    A[i].f2 = j;
                                    break;
                                default:
                                    printf( "`A_from_VF`: THIS SHOULD NOT OCCUR!\n" );
                                    break;
                            }
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
    printf( "\tThere were %i comparisons made! \n\n", totCompr );
}


bool p_net_faces_outward_convex( uint Ntri_, uint Nvrt_, const vec4f* V, const vec3u* F, const vec4f* N ){
    // Return true if the face normals N of an (assumed convex) net all point away from the centroid of vertices
    vec4f total = {0.0f,0.0f,0.0f,1.0f};
    vec4f centr = {0.0f,0.0f,0.0f,1.0f};
    vec4f v0_i = {0.0f,0.0f,0.0f,1.0f};
    vec4f nrm_i = {0.0f,0.0f,0.0f,1.0f};
    float dTest = 0.0f;
    bool  allOutward = true;
    // Calc Centroid //
    for( uint i = 0 ; i < Nvrt_ ; ++i ){
        total = add_vec4f( total, V[i] );
    }
    centr = div_vec4f( total, (float) Nvrt_ );
    // Check Each Normal //
    for( uint i = 0 ; i < Ntri_ ; ++i ){
        v0_i  = V[ F[i].v0 ];
        nrm_i = N[ F[i].v0 ];
        dTest = dot_vec4f( sub_vec4f( v0_i, centr ), nrm_i );
        // printf( "%f, ", dTest );
        if( dTest < 0.0f ){
            printf( "Face %i should be REVERSED!\n", i );
            allOutward = false;
        }
    }
    if( allOutward ){  printf( "ALL triangles OKAY!\n" );  }
    return allOutward;
}


void populate_net_connectivity( TriNet* net, float eps ){
    // Get facet neighborhoods 
    A_from_VF( net->A, net->Ntri, eps, net->V, net->F );
}


void draw_net_wireframe( TriNet* net, vec4f lineColor ){
    // Draw the net as a wireframe, NOTE: Only `V` and `F` data req'd
    glClr4f( lineColor );
    glBegin( GL_LINES );
    for( uint i = 0; i < net->Ntri; ++i ){
        glVtx4f( net->V[ net->F[i].v0 ] );  glVtx4f( net->V[ net->F[i].v1 ] );  
        glVtx4f( net->V[ net->F[i].v1 ] );  glVtx4f( net->V[ net->F[i].v2 ] );  
        glVtx4f( net->V[ net->F[i].v2 ] );  glVtx4f( net->V[ net->F[i].v0 ] );  
    }
    glEnd();
}


void draw_net_connectivity( TriNet* net, vec4f lineColor ){
    // Draw the net neighbors as a wireframe
    vec4f triCntr = {0.0f,0.0f,0.0f,1.0f};
    vec4f segCntr;
    glClr4f( lineColor );
    glBegin( GL_LINES );
    // 1. For each F
    for( uint i = 0; i < net->Ntri; ++i ){
        // 2. Calc center
        triCntr = tri_center( net->V[ net->F[i].v0 ], net->V[ net->F[i].v1 ], net->V[ net->F[i].v2 ] );
        // 4. Test each possible connection. If it exists, then draw a segment from center of F to center of shared edge
        if( net->A[i].f0 != i ){
            seg_center( &segCntr, &v0, &v1 );
            glVtx3f( triCntr );  glVtx3f( segCntr );  
        }
        if( net->A[i].f1 != i ){
            seg_center( &segCntr, &v1, &v2 );
            glVtx3f( triCntr );  glVtx3f( segCntr );  
        }
        if( net->A[i].f2 != i ){
            seg_center( &segCntr, &v2, &v0 );
            glVtx3f( triCntr );  glVtx3f( segCntr );  
        }
    }
    glEnd();
}



////////// SPECIFIC POLYHEDRA //////////////////////////////////////////////////////////////////////

///// Tetrahedron /////////////////////////////////////////////////////////

void populate_tetra_vertices_and_faces( matx_Nx3f* V, matx_Nx3u* F, float radius ){
	// Load geometry for an icosahedron onto matrices `V` and `F` 
	/// Calc req'd constants ///
    float a = radius;
    float b = radius / sqrtf( 2.0f );
	/// Load Vertices ///
	// Assume `V` already allocated for *4* vertices
    load_row_from_3f( V, 0,   a   , 0.0f,-b );
    load_row_from_3f( V, 1,  -a   , 0.0f,-b );
    load_row_from_3f( V, 2,   0.0f, a   , b );
    load_row_from_3f( V, 3,   0.0f,-a   , b );
	/// Load Faces ///
	// Assume `F` already allocated for *4* faces
	load_row_from_3u( F, 0,  0,1,2 );
	load_row_from_3u( F, 1,  0,2,3 );
	load_row_from_3u( F, 2,  0,3,1 );
	load_row_from_3u( F, 3,  1,3,2 );
}

TriNet* create_tetra_mesh_only( float radius ){
	// Create an regular icosahedron (*without* unfolded net data)
	/// Allocate ///
	TriNet* tetraNet = alloc_net( 4, 4 );
	/// Vertices and Faces ///
	populate_tetra_vertices_and_faces( tetraNet->V, tetraNet->F, radius );
	/// Normals ///
	N_from_VF( tetraNet->Ntri, tetraNet->V, tetraNet->F, tetraNet->N );
	/// Return ///
	return tetraNet;
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
    load_row_from_3f( V, 0,  0, b,-a ); 
    load_row_from_3f( V, 1,  b, a, 0 );
    load_row_from_3f( V, 2, -b, a, 0 );
    load_row_from_3f( V, 3,  0, b, a );
    load_row_from_3f( V, 4,  0,-b, a );
    load_row_from_3f( V, 5, -a, 0, b );
    load_row_from_3f( V, 6,  0,-b,-a );
    load_row_from_3f( V, 7,  a, 0,-b );
    load_row_from_3f( V, 8,  a, 0, b );
    load_row_from_3f( V, 9, -a, 0,-b );
    load_row_from_3f( V,10,  b,-a, 0 );
    load_row_from_3f( V,11, -b,-a, 0 );
    /// Load Faces ///
    // Assume `F` already allocated for *20* faces
    load_row_from_3u( F, 0,  2, 1, 0 );
    load_row_from_3u( F, 1,  1, 2, 3 );
    load_row_from_3u( F, 2,  5, 4, 3 );
    load_row_from_3u( F, 3,  4, 8, 3 );
    load_row_from_3u( F, 4,  7, 6, 0 );
    load_row_from_3u( F, 5,  6, 9, 0 );
    load_row_from_3u( F, 6, 11,10, 4 );
    load_row_from_3u( F, 7, 10,11, 6 );
    load_row_from_3u( F, 8,  9, 5, 2 );
    load_row_from_3u( F, 9,  5, 9,11 );
    load_row_from_3u( F,10,  8, 7, 1 );
    load_row_from_3u( F,11,  7, 8,10 );
    load_row_from_3u( F,12,  2, 5, 3 );
    load_row_from_3u( F,13,  8, 1, 3 );
    load_row_from_3u( F,14,  9, 2, 0 );
    load_row_from_3u( F,15,  1, 7, 0 );
    load_row_from_3u( F,16, 11, 9, 6 );
    load_row_from_3u( F,17,  7,10, 6 );
    load_row_from_3u( F,18,  5,11, 4 );
    load_row_from_3u( F,19, 10, 8, 4 );
}


TriNet* create_icos_mesh_only( float radius ){
    // Create an regular icosahedron (*without* unfolded net data)
    /// Allocate ///
    TriNet* icosNet = alloc_net( 20, 12 );
    /// Vertices and Faces ///
    populate_icos_vertices_and_faces( icosNet->V, icosNet->F, radius );
    /// Normals ///
    N_from_VF( icosNet->Ntri, icosNet->V, icosNet->F, icosNet->N );
    /// Return ///
    return icosNet;
}


TriNet* create_icos_VFNA( float radius ){
    // Create an regular icosahedron (*without* unfolded net data)
    /// Allocate ///
    TriNet* icosNet = alloc_net( 20, 12 );
    /// Vertices and Faces ///
    populate_icos_vertices_and_faces( icosNet->V, icosNet->F, radius );
    /// Normals ///
    N_from_VF( icosNet->Ntri, icosNet->V, icosNet->F, icosNet->N );
    /// Advacency ///
    populate_net_connectivity( icosNet, 0.005 );
    /// Return ///
    return icosNet;
}



///// Sphere from Divided Icos ////////////////////////////////////////////

TriNet* create_icosphere_mesh_only( float radius, uint div ){
    // Construct a sphere with `radius` from a subdivided icos (`div` rows) and center at {0,0,0}
    vec3f v0, v1, v2, xTri, yTri, vA, vB, vC, nA, nB, nC, nT;
    vec2f   vct2f;
    uint    Ntri = 20 * (div*(div+1)/2 + (div-1)*(div)/2);
    uint    Nvrt = Ntri * 3;
    TriNet* icos = create_icos_mesh_only( radius );
    TriNet* sphr = alloc_net( Ntri, Nvrt );
    uint    k    = 0;
    uint    m    = 0;
    uint    swap = 0;
    // 1. For every triangle in the icos, Load geo data
    for( ubyte i = 0; i < 20; ++i ){
        load_vec3f_from_row( &v0, icos->V, (*icos->F)[i][0] );
        load_vec3f_from_row( &v1, icos->V, (*icos->F)[i][1] );
        load_vec3f_from_row( &v2, icos->V, (*icos->F)[i][2] );
        sub_vec3f( &xTri, &v1, &v0 );  scale_vec3f( &xTri, &xTri, 1.0f/((float)div) );
        sub_vec3f( &yTri, &v2, &v0 );  scale_vec3f( &yTri, &yTri, 1.0f/((float)div) );
        // 1. For every subdivided row of this triangle, Do ...
        for( uint row = 1; row <= div; ++row ){
            // 2. Construct the v0-pointing triangles of this row
            for( uint j = row ; j > 0 ; j-- ){ 
                vct2f[0] = (float) (j  );
                vct2f[1] = (float) (row-j);
                lift_pnt_2D_to_3D_vec3f( &vA, &vct2f, &v0, &xTri, &yTri );
                vct2f[0] = (float) (j-1);
                vct2f[1] = (float) (row-j+1);
                lift_pnt_2D_to_3D_vec3f( &vB, &vct2f, &v0, &xTri, &yTri );
                vct2f[0] = (float) (j-1);
                vct2f[1] = (float) (row-j);
                lift_pnt_2D_to_3D_vec3f( &vC, &vct2f, &v0, &xTri, &yTri );
                unit_vec3f( &nA, &vA );
                unit_vec3f( &nB, &vB );
                unit_vec3f( &nC, &vC );
                scale_vec3f( &vA, &nA, radius );
                scale_vec3f( &vB, &nB, radius );
                scale_vec3f( &vC, &nC, radius );
                load_row_from_vec3f( sphr->V, k, &vA );  (*sphr->F)[m][0] = k;  ++k;
                load_row_from_vec3f( sphr->V, k, &vB );  (*sphr->F)[m][1] = k;  ++k;
                load_row_from_vec3f( sphr->V, k, &vC );  (*sphr->F)[m][2] = k;  ++k;
                // 3. Enforce triangle convex
                get_CCW_tri_norm( &nT, &vA, &vB, &vC );
                if( dot_vec3f( &nT, &nA ) < 0.0f ){
                    swap /*-------*/ = (*sphr->F)[m][2];
                    (*sphr->F)[m][2] = (*sphr->F)[m][1];
                    (*sphr->F)[m][1] = swap;
                }
                ++m; // Next triangle
            }
            // 4. Construct the anti-v0-pointing triangles for this row
            for( ubyte j = row - 1 ; j > 0 ; j-- ){ 
                vct2f[0] = (float) (j);
                vct2f[1] = (float) (row-1-j);
                lift_pnt_2D_to_3D_vec3f( &vA, &vct2f, &v0, &xTri, &yTri );
                vct2f[0] = (float) (j);
                vct2f[1] = (float) (row-1-j+1);
                lift_pnt_2D_to_3D_vec3f( &vB, &vct2f, &v0, &xTri, &yTri );
                vct2f[0] = (float) (j-1);
                vct2f[1] = (float) (row-1-j+1);
                lift_pnt_2D_to_3D_vec3f( &vC, &vct2f, &v0, &xTri, &yTri );
                unit_vec3f( &nA, &vA );
                unit_vec3f( &nB, &vB );
                unit_vec3f( &nC, &vC );
                scale_vec3f( &vA, &nA, radius );
                scale_vec3f( &vB, &nB, radius );
                scale_vec3f( &vC, &nC, radius );
                load_row_from_vec3f( sphr->V, k, &vA );  (*sphr->F)[m][0] = k;  ++k;
                load_row_from_vec3f( sphr->V, k, &vB );  (*sphr->F)[m][1] = k;  ++k;
                load_row_from_vec3f( sphr->V, k, &vC );  (*sphr->F)[m][2] = k;  ++k;
                // 5. Enforce triangle convex
                get_CCW_tri_norm( &nT, &vA, &vB, &vC );
                if( dot_vec3f( &nT, &nA ) < 0.0f ){
                    swap /*-------*/ = (*sphr->F)[m][2];
                    (*sphr->F)[m][2] = (*sphr->F)[m][1];
                    (*sphr->F)[m][1] = swap;
                }
                ++m; // Next triangle
            }
        }
    }
    // 6. Erase base icos
    delete_net( icos );
    // 7. Return mesh
    return sphr;
}


TriNet* create_icosphere_VFNA( float radius, uint div ){
    // Create an regular icosahedron (*without* unfolded net data)
    /// Allocate Vertices and Faces ///
    TriNet* sphrNet = create_icosphere_mesh_only( radius, div );
    /// Normals ///
    N_from_VF( sphrNet->Ntri, sphrNet->V, sphrNet->F, sphrNet->N );
    // repair_net_faces_outward_convex( 
    //     sphrNet->Ntri, 
    //     sphrNet->Nvrt, 
    //     sphrNet->V, 
    //     sphrNet->F, 
    //     sphrNet->N 
    // );
    /// Advacency ///
    populate_net_connectivity( sphrNet, 0.005 );
    /// Return ///
    return sphrNet;
}

#endif
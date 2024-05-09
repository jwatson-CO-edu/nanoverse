

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"



////////// POLYHEDRAL NET //////////////////////////////////////////////////////////////////////////


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
    // Load a uint array from a `vec3u` struct
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
                        vert_j1 = V[ face_j[b      ] ];
                        vert_j2 = V[ face_j[(b+1)%3] ];
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
    vec4f triCntr = make_0_vec4f();
    vec4f segCntr = make_0_vec4f();
    vec4f v0 = make_0_vec4f();
    vec4f v1 = make_0_vec4f();
    vec4f v2 = make_0_vec4f();
    glClr4f( lineColor );
    glBegin( GL_LINES );
    // 1. For each F
    for( uint i = 0; i < net->Ntri; ++i ){
        // 2. Calc center
        v0 = net->V[ net->F[i].v0 ];
        v1 = net->V[ net->F[i].v1 ];
        v2 = net->V[ net->F[i].v2 ];
        triCntr = tri_center( v0, v1, v2 );
        // 4. Test each possible connection. If it exists, then draw a segment from center of F to center of shared edge
        if( net->A[i].f0 != i ){
            segCntr = seg_center( v0, v1 );
            glVtx4f( triCntr );  glVtx4f( segCntr );  
        }
        if( net->A[i].f1 != i ){
            segCntr = seg_center( v1, v2 );
            glVtx4f( triCntr );  glVtx4f( segCntr );  
        }
        if( net->A[i].f2 != i ){
            segCntr = seg_center( v2, v0 );
            glVtx4f( triCntr );  glVtx4f( segCntr );  
        }
    }
    glEnd();
}



////////// SPECIFIC POLYHEDRA //////////////////////////////////////////////////////////////////////

///// Tetrahedron /////////////////////////////////////////////////////////

void populate_tetra_vertices_and_faces( vec4f* V, vec3u* F, float radius ){
	// Load geometry for an icosahedron onto matrices `V` and `F` 
	/// Calc req'd constants ///
    float a = radius;
    float b = radius / sqrtf( 2.0f );
	/// Load Vertices ///
	// Assume `V` already allocated for *4* vertices
    V[0] = make_vec4f(  a   , 0.0f,-b );
    V[1] = make_vec4f( -a   , 0.0f,-b );
    V[2] = make_vec4f(  0.0f, a   , b );
    V[3] = make_vec4f(  0.0f,-a   , b );
	/// Load Faces ///
	// Assume `F` already allocated for *4* faces
    F[0] = make_vec3u( 0,1,2 );
    F[1] = make_vec3u( 0,2,3 );
    F[2] = make_vec3u( 0,3,1 );
    F[3] = make_vec3u( 1,3,2 );
}

TriNet* create_tetra_mesh_only( float radius ){
	// Create an regular icosahedron (*without* unfolded net data)
	/// Allocate ///
	TriNet* tetraNet = alloc_net( 4, 4 );
	/// Vertices and Faces ///
	populate_tetra_vertices_and_faces( tetraNet->V, tetraNet->F, radius );
	/// Normals ///
	N_from_VF( tetraNet->N, tetraNet->Ntri, tetraNet->V, tetraNet->F );
	/// Return ///
	return tetraNet;
}



///// Icosahedron /////////////////////////////////////////////////////////

void populate_icos_vertices_and_faces( vec4f* V, vec3u* F, float radius ){
    // Load geometry for an icosahedron onto matrices `V` and `F` 
    /// Calc req'd constants ///
    float sqrt5 = (float) sqrt( 5.0 ); // ----------------------------------- Square root of 5
    float phi   = (float)( 1.0 + sqrt5 ) * 0.5; // ------------------------- The Golden Ratio
    float ratio = (float)sqrt( 10.0 + ( 2.0 * sqrt5 ) ) / ( 4.0 * phi ); // ratio of edge length to radius
    float a     = ( radius / ratio ) * 0.5;
    float b     = ( radius / ratio ) / ( 2.0f * phi );
    /// Load Vertices ///
    // Assume `V` already allocated for *12* vertices
    V[ 0] = make_vec4f(  0, b,-a );
    V[ 1] = make_vec4f(  b, a, 0 );
    V[ 2] = make_vec4f( -b, a, 0 );
    V[ 3] = make_vec4f(  0, b, a );
    V[ 4] = make_vec4f(  0,-b, a );
    V[ 5] = make_vec4f( -a, 0, b );
    V[ 6] = make_vec4f(  0,-b,-a );
    V[ 7] = make_vec4f(  a, 0,-b );
    V[ 8] = make_vec4f(  a, 0, b );
    V[ 9] = make_vec4f( -a, 0,-b );
    V[10] = make_vec4f(  b,-a, 0 );
    V[11] = make_vec4f( -b,-a, 0 );

    /// Load Faces ///
    // Assume `F` already allocated for *20* faces
    F[ 0] = make_vec3u(  2, 1, 0 );
    F[ 1] = make_vec3u(  1, 2, 3 );
    F[ 2] = make_vec3u(  5, 4, 3 );
    F[ 3] = make_vec3u(  4, 8, 3 );
    F[ 4] = make_vec3u(  7, 6, 0 );
    F[ 5] = make_vec3u(  6, 9, 0 );
    F[ 6] = make_vec3u( 11,10, 4 );
    F[ 7] = make_vec3u( 10,11, 6 );
    F[ 8] = make_vec3u(  9, 5, 2 );
    F[ 9] = make_vec3u(  5, 9,11 );
    F[10] = make_vec3u(  8, 7, 1 );
    F[11] = make_vec3u(  7, 8,10 );
    F[12] = make_vec3u(  2, 5, 3 );
    F[13] = make_vec3u(  8, 1, 3 );
    F[14] = make_vec3u(  9, 2, 0 );
    F[15] = make_vec3u(  1, 7, 0 );
    F[16] = make_vec3u( 11, 9, 6 );
    F[17] = make_vec3u(  7,10, 6 );
    F[18] = make_vec3u(  5,11, 4 );
    F[19] = make_vec3u( 10, 8, 4 );
}


TriNet* create_icos_mesh_only( float radius ){
    // Create an regular icosahedron (*without* unfolded net data)
    /// Allocate ///
    TriNet* icosNet = alloc_net( 20, 12 );
    /// Vertices and Faces ///
    populate_icos_vertices_and_faces( icosNet->V, icosNet->F, radius );
    /// Normals ///
    N_from_VF( icosNet->N, icosNet->Ntri, icosNet->V, icosNet->F );
    /// Return ///
    return icosNet;
}


TriNet* create_icos_VFNA( float radius ){
    // Create an regular icosahedron (*with* unfolded net data)
    /// Allocate ///
    TriNet* icosNet = alloc_net( 20, 12 );
    /// Vertices and Faces ///
    populate_icos_vertices_and_faces( icosNet->V, icosNet->F, radius );
    /// Normals ///
    N_from_VF( icosNet->N, icosNet->Ntri, icosNet->V, icosNet->F );
    /// Advacency ///
    populate_net_connectivity( icosNet, radius/50.0f );
    /// Return ///
    return icosNet;
}



///// Sphere from Divided Icos ////////////////////////////////////////////

TriNet* create_icosphere_mesh_only( float radius, uint div ){
    // Construct a sphere with `radius` from a subdivided icos (`div` rows) and center at {0,0,0}
    vec4f v0, v1, v2, xTri, yTri, vA, vB, vC, nT;
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
        v0 = icos->V[ icos->F[i].v0 ];
        v1 = icos->V[ icos->F[i].v1 ];
        v2 = icos->V[ icos->F[i].v2 ];
        xTri = scale_vec4f( sub_vec4f( v1, v0 ), 1.0f/((float)div) );
        yTri = scale_vec4f( sub_vec4f( v2, v0 ), 1.0f/((float)div) );
        // 1. For every subdivided row of this triangle, Do ...
        for( uint row = 1; row <= div; ++row ){
            // 2. Construct the v0-pointing triangles of this row
            for( uint j = row ; j > 0 ; j-- ){ 
                vct2f = make_vec2f( (float) (j  ), (float) (row-j) );
                vA = lift_pnt_2D_to_3D( vct2f, v0, xTri, yTri );
                vct2f = make_vec2f( (float) (j-1), (float) (row-j+1) );
                vB = lift_pnt_2D_to_3D( vct2f, v0, xTri, yTri );
                vct2f = make_vec2f( (float) (j-1), (float) (row-j) );
                vC = lift_pnt_2D_to_3D( vct2f, v0, xTri, yTri );
                vA = scale_vec4f( unit_vec4f( vA ), radius );
                vB = scale_vec4f( unit_vec4f( vB ), radius );
                vC = scale_vec4f( unit_vec4f( vC ), radius );
                sphr->V[k] = vA;  sphr->F[m].v0 = k;  ++k;
                sphr->V[k] = vB;  sphr->F[m].v1 = k;  ++k;
                sphr->V[k] = vC;  sphr->F[m].v2 = k;  ++k;
                // 3. Enforce triangle convex
                nT = get_CCW_tri_norm( vA, vB, vC );
                if( dot_vec4f( nT, vA ) < 0.0f ){
                    swap /*----*/ = sphr->F[m].v2;
                    sphr->F[m].v2 = sphr->F[m].v1;
                    sphr->F[m].v1 = swap;
                }
                ++m; // Next triangle
            }
            // 4. Construct the anti-v0-pointing triangles for this row
            for( ubyte j = row - 1 ; j > 0 ; j-- ){ 
                vct2f = make_vec2f( (float) (j), (float) (row-1-j) );
                vA = lift_pnt_2D_to_3D( vct2f, v0, xTri, yTri );
                vct2f = make_vec2f( (float) (j), (float) (row-1-j+1) );
                vB = lift_pnt_2D_to_3D( vct2f, v0, xTri, yTri );
                vct2f = make_vec2f( (float) (j-1), (float) (row-1-j+1) );
                vC = lift_pnt_2D_to_3D( vct2f, v0, xTri, yTri );
                vA = scale_vec4f( unit_vec4f( vA ), radius );
                vB = scale_vec4f( unit_vec4f( vB ), radius );
                vC = scale_vec4f( unit_vec4f( vC ), radius );
                sphr->V[k] = vA;  sphr->F[m].v0 = k;  ++k;
                sphr->V[k] = vB;  sphr->F[m].v1 = k;  ++k;
                sphr->V[k] = vC;  sphr->F[m].v2 = k;  ++k;
                // 5. Enforce triangle convex
                nT = get_CCW_tri_norm( vA, vB, vC );
                if( dot_vec4f( nT, vA ) < 0.0f ){
                    swap /*-------*/ = sphr->F[m].v2;
                    sphr->F[m].v2 = sphr->F[m].v1;
                    sphr->F[m].v1 = swap;
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
    // Create an regular icosahedron (*with* unfolded net data)
    /// Allocate Vertices and Faces ///
    TriNet* sphrNet = create_icosphere_mesh_only( radius, div );
    /// Normals ///
    N_from_VF( sphrNet->N, sphrNet->Ntri, sphrNet->V, sphrNet->F );
    /// Advacency ///
    populate_net_connectivity( sphrNet, radius/50.0f );
    /// Return ///
    return sphrNet;
}

#include "include/geometry.hpp"

////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////

TriNet::TriNet( uint Ntri_, uint Nvrt_, bool textured = false ){
    // FIXME: SWITCH TO LAZY MODEL
    Nvrt    = Nvrt_; // ------- Number of vertices
    Ntri    = Ntri_; // ------- Number of triangles
    V /*-*/ = nullptr; // ----- Vertices: _________ Nvrt X {x,y,z}
    F /*-*/ = nullptr; // ----- Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    N /*-*/ = nullptr; // ----- Face Normals: _____ Nvrt X {x,y,z}
    A /*-*/ = nullptr; // ----- Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order (Calc lazily)
    p_tex   = textured; // ---- Flag: Is a texture being used?
    bufID   = UINT_MAX; // ---- Buffer ID at the GPU
    relPose = mat4f{ 1.0f }; // Static offset pose from parent pose (anchor)
    ownPose = mat4f{ 1.0f }; // Dynamic offset pose from `relPose`
    T /*-*/ = nullptr;
    C /*-*/ = nullptr;
}


TriNet::~TriNet(){
    free(V); // ----------- Vertices: _________ Nvrt X {x,y,z}
    free(F); // ----------- Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    free(N); // ----------- Vert Normals: _____ Nvrt X {x,y,z}, Normal is the "Zdir" of local coord system
    if(A){  free(A);  } //- Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
    if(T){  free(T);  }; // Texture Coords: ___ Nvrt X {u,v}
    if(C){  free(C);  }; // Color Coords: _____ Nvrt X {x,y,z}
};


void TriNet::add_tri( const tri_v4f& tri ){
    uint k = verts.size()*3;;
    for( ubyte i = 0; i < 3; ++i ){
        verts.push_back( vec4f{ tri[i][0], tri[i][1], tri[i][2], tri[i][3] } );
        faces.push_back( vec3u{ k+0, k+1, k+2 } );
        k += 3;
    }
}


void TriNet::add_tri_clr( const tri_v4f& tri, const tri_v4f& clr ){
    uint k = verts.size()*3;;
    for( ubyte i = 0; i < 3; ++i ){
        verts.push_back( vec4f{ tri[i][0], tri[i][1], tri[i][2], tri[i][3] } );
        colrs.push_back( vec4f{ clr[i][0], clr[i][1], clr[i][2], clr[i][3] } );
        faces.push_back( vec3u{ k+0, k+1, k+2 } );
        k += 3;
    }
}


void TriNet::add_tri_clr( const tri_v4f& tri, const vec4f& clr ){  add_tri_clr( tri, tri_v4f{ clr, clr, clr } );  }


vec3f no_scale( const vec4f& vec ){  return vec3f{ vec.x, vec.y, vec.z };  }


vec4f extend( const vec3f& vec ){  return vec4f{ vec[0], vec[1], vec[2], 1.0f };  } // Estend to 4 elems, Unity scale


vec4f cross_vec4f( vec4f _v1, vec4f _v2 ){
    // Cross two `vec4f` as though they were `vec3f`
    return extend( cross( vec3f{ _v1[0], _v1[1], _v1[2] }, vec3f{ _v2[0], _v2[1], _v2[2] } ) );
}


vec4f get_normal( const tri_v4f& tri ){
    vec3f v0 = no_scale( tri[1] ) - no_scale( tri[0] );
    vec3f v1 = no_scale( tri[2] ) - no_scale( tri[0] );
    return extend( normalize( cross( v0, v1 ) ) );
}


float diff_norm( const vec4f& v1, const vec4f& v2 ){  return length( no_scale( v1 ) - no_scale( v2 ) );  }


void TriNet::store_geo( float eps ){
    // Compress shared vertices and store in compact heap arrays
    size_t /**/ Nvtx = verts.size();
    size_t /**/ Nfac = faces.size();
    vec4f /*-*/ vec_i;
    vec4f /*-*/ vec_j;
    vector<vec4f> vVerts = list_2_vector( verts );
    vector<vec3u> vFaces = list_2_vector( faces );
    for( size_t i = 0; i < (Nvtx-1); ++i ){
        vec_i = vVerts[i];
        for( size_t j = i+1; j < Nvtx; ++j ){
            vec_j = vVerts[j];
            if( diff_norm( vec_i, vec_j ) < eps ){
                for( size_t k = 0; k < Nfac; ++k ){
                    for( size_t l = 0; l < 3; ++l ){  if( vFaces[k][l] == j )  vFaces[k][l] = i;  }   
                }
            }
        }   
    }
} 
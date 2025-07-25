#include "geometry.hpp"

////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////

TriNet::TriNet( uint Ntri_, uint Nvrt_, bool textured = false ){
    // FIXME: SWITCH TO LAZY MODEL
    Nvrt    = Nvrt_; // ------------------------------------ Number of vertices
    Ntri    = Ntri_; // ------------------------------------ Number of triangles
    V /*-*/ = (vec4f*) malloc( Nvrt_ * sizeof( vec4f ) ); // Vertices: _________ Nvrt X {x,y,z}
    F /*-*/ = (vec3u*) malloc( Ntri_ * sizeof( vec3u ) ); // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    N /*-*/ = (vec4f*) malloc( Nvrt_ * sizeof( vec4f ) ); // Face Normals: _____ Nvrt X {x,y,z}
    A /*-*/ = nullptr; // ---------------------------------- Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order (Calc lazily)
    p_tex   = textured; // --------------------------------- Flag: Is a texture being used?
    bufID   = UINT_MAX; // --------------------------------- Buffer ID at the GPU
    relPose = mat4f(); // ---------------------------------- Static offset pose from parent pose (anchor)
    ownPose = mat4f(); // ---------------------------------- Dynamic offset pose from `relPose`
    if( textured ){
        T = (vec2f*) malloc( Nvrt_ * sizeof( vec2f ) );
        C = nullptr;
    }else{
        T = nullptr;
        C = (vec4f*) malloc( Nvrt_ * sizeof( vec4f ) );
    }
}


TriNet::~TriNet(){
    free(V); // ----------- Vertices: _________ Nvrt X {x,y,z}
    free(F); // ----------- Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    free(N); // ----------- Vert Normals: _____ Nvrt X {x,y,z}, Normal is the "Zdir" of local coord system
    if(A){  free(A);  } //  Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
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


void TriNet::calc_flat_shading_normals(){

}
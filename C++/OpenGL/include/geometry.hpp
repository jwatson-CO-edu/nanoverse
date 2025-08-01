#ifndef GEOMETRY_HPP // This pattern is to prevent symbols to be loaded multiple times
#define GEOMETRY_HPP // from multiple imports

#include <list>
using std::list;
#include <array>
using std::array;

#include "toolbox.hpp"

typedef array<vec4f,3> tri_v4f;
typedef array<vec3f,3> tri_v3f;




////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

vec4f extend( const vec3f& vec ); // Estend to 4 elems, Unity scale
vec4f cross_vec4f( vec4f _v1, vec4f _v2 ); // Cross two `vec4f` as though they were `vec3f`



///// Polyhedral Net //////////////////////////////////////////////////////
class TriNet;
typedef shared_ptr<TriNet> NetPtr;

class TriNet{ public:
    // Holds geo info for a polyhedral net of triangles // WARNING: THERE SHOULD BE A NORMAL FOR EACH VERTEX
    uint   Nvrt; // --------- Number of vertices
    uint   Ntri; // --------- Number of triangles
    vec4f* V; // ------------ Vertices: _________ Nvrt X {x,y,z}
    vec3u* F; // ------------ Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    vec4f* N; // ------------ Vert Normals: _____ Nvrt X {x,y,z}, Normal is the "Zdir" of local coord system
    vec3u* A; // ------------ Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
    vec2f* T; // ------------ Texture Coords: ___ Nvrt X {u,v}
    vec4f* C; // ------------ Color Coords: _____ Nvrt X {x,y,z}
    bool   p_tex; // -------- Flag: Is a texture being used?
    uint   bufID; // -------- Buffer ID at the GPU
    float  eps_m = 0.0005; // Margin to merge vertices
    /// Pose & Scale ///
    mat4f relPose; // Static offset pose from parent pose (anchor)
    mat4f ownPose; // Dynamic offset pose from `relPose`
    /// Structure ///
    list<vec4f>    verts; // Unshared vertices
    list<vec4f>    colrs; // Unshared vertices
    list<vec4f>    norms; // Unshared vertices
    list<vec3u>    faces; // Sequential faces
    vector<NetPtr> edges; // Links to successor nodes, Need contant-time ACCESS!, Assumed not to change at render time

    TriNet( uint Ntri_, uint Nvrt_, bool textured ); // FIXME: SWITCH TO LAZY MODEL
    ~TriNet();

    void add_tri( const tri_v4f& tri );
    void add_tri_clr( const tri_v4f& tri, const tri_v4f& clr );
    void add_tri_clr( const tri_v4f& tri, const vec4f& clr   );
    void store_geo( float eps = 0.005 ); // Compress shared vertices and store in compact heap arrays

    void load_tetra( float radius = 1.0f ); // --- Replace geo with tetrahedron
    void load_icos( float radius = 1.0f ); // ---- Replace geo with icosahedron
    void load_icosphere( float radius = 1.0f ); // Replace geo with subdivided icosahedron
};

#endif
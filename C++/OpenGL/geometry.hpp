#ifndef GEOMETRY_HPP // This pattern is to prevent symbols to be loaded multiple times
#define GEOMETRY_HPP // from multiple imports

#include "toolbox.hpp"

////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////
class TriNet;
typedef shared_ptr<TriNet> NetPtr;

class TriNet{ public:
    // Holds geo info for a polyhedral net of triangles // WARNING: THERE SHOULD BE A NORMAL FOR EACH VERTEX
    uint   Nvrt; // Number of vertices
    uint   Ntri; // Number of triangles
    vec4f* V; // -- Vertices: _________ Nvrt X {x,y,z}
    vec3u* F; // -- Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    vec4f* N; // -- Vert Normals: _____ Nvrt X {x,y,z}, Normal is the "Zdir" of local coord system
    vec3u* A; // -- Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
    vec2f* T; // -- Texture Coords: ___ Nvrt X {u,v}
    vec4f* C; // -- Color Coords: _____ Nvrt X {x,y,z}
    bool   p_tex; // Flag: Is a texture being used?
    uint   bufID; // Buffer ID at the GPU
    /// Pose & Scale ///
    mat4f relPose; // Static offset pose from parent pose (anchor)
    mat4f ownPose; // Dynamic offset pose from `relPose`
    /// Structure ///
    vector<NetPtr> edges; // Links to successor nodes, Need contant-time ACCESS!, Assumed not to change at render time

    TriNet( uint Ntri_, uint Nvrt_, bool textured );
    ~TriNet();
};

#endif
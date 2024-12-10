#ifndef GEOMETRY_HPP // This pattern is to prevent symbols to be loaded multiple times
#define GEOMETRY_HPP // from multiple imports

#include "toolbox.hpp"

////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////

class TriNet{ public:
    // Holds geo info for a polyhedral net of triangles // WARNING: THERE SHOULD BE A NORMAL FOR EACH VERTEX
    uint   Nvrt; // Number of vertices
    uint   Ntri; // Number of triangles
    vec4f* V; // Vertices: _________ Nvrt X {x,y,z}
    vec3u* F; // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    vec4f* N; // Vert Normals: _____ Nvrt X {x,y,z}, Normal is the "Zdir" of local coord system
    vec3u* A; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
};

#endif
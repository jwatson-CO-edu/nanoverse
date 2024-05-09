#ifndef GEOMETRY_H // This pattern is to prevent symbols to be loaded multiple times
#define GEOMETRY_H // from multiple imports

#include "toolbox.h"

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



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// TriNet.c ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// POLYHEDRAL NET //////////////////////////////////////////////////////////////////////////

// Allocate mem for a `TriNet` with `Ntri_` faces and `Nvrt_` vertices (shared vertices allowed)
TriNet* alloc_net( uint Ntri_, uint Nvrt_ );
void delete_net( TriNet* net ); // Free mem for a `TriNet` 
void N_from_VF( vec4f* N, /*<<*/ uint Ntri_, const vec4f* V, const vec3u* F ); // Calc all F normals (One per F)
void set_uintArr3_from_vec3u( uint* arr, const vec3u vec ); // Load a uint array from a `vec3u` struct
// Find F adjacencies and store connectivity in `A`, O(n^2) in number of faces
void A_from_VF( vec3u* A, /*<<*/ uint Ntri_, float eps, const vec4f* V, const vec3u* F );
// Return true if the face normals N of an (assumed convex) net all point away from the centroid of vertices
bool p_net_faces_outward_convex( uint Ntri_, uint Nvrt_, const vec4f* V, const vec3u* F, const vec4f* N );
void populate_net_connectivity( TriNet* net, float eps ); // Get facet neighborhoods 
void draw_net_wireframe( TriNet* net, vec4f lineColor ); // Draw the net as a wireframe, NOTE: Only `V` and `F` data req'd
void draw_net_connectivity( TriNet* net, vec4f lineColor ); // Draw the net neighbors as a wireframe


////////// SPECIFIC POLYHEDRA //////////////////////////////////////////////////////////////////////

///// Tetrahedron /////////////////////////////////////////////////////////
// Load geometry for an tetrahedron onto matrices `V` and `F` 
void populate_tetra_vertices_and_faces( vec4f* V, vec3u* F, float radius );
TriNet* create_tetra_mesh_only( float radius ); // Create an regular tetrahedron (*without* unfolded net data)

///// Icosahedron /////////////////////////////////////////////////////////
// Load geometry for an icosahedron onto matrices `V` and `F` 
void populate_icos_vertices_and_faces( vec4f* V, vec3u* F, float radius );
TriNet* create_icos_mesh_only( float radius ); // Create an regular icosahedron (*without* unfolded net data)
TriNet* create_icos_VFNA( float radius ); // Create an regular icosahedron (*with* unfolded net data)

///// Sphere from Divided Icos ////////////////////////////////////////////
// Construct a sphere with `radius` from a subdivided icos (`div` rows) and center at {0,0,0}
TriNet* create_icosphere_mesh_only( float radius, uint div );
TriNet* create_icosphere_VFNA( float radius, uint div ); // Create an regular icosahedron (*with* unfolded net data)
    
    
    

#endif
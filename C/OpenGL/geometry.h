#ifndef GEOMETRY_H // This pattern is to prevent symbols to be loaded multiple times
#define GEOMETRY_H // from multiple imports

#include "toolbox.h"

////////// GEOMETRY STRUCTS ////////////////////////////////////////////////////////////////////////

///// Polyhedral Net //////////////////////////////////////////////////////

typedef struct{
    // Holds geo info for a polyhedral net of triangles // WARNING: THERE SHOULD BE A NORMAL FOR EACH VERTEX
    uint   Nvrt; // Number of vertices
    uint   Ntri; // Number of triangles
    vec4f* V; // Vertices: _________ Nvrt X {x,y,z}
    vec3u* F; // Faces: ____________ Ntri X {v1,v2,v3}, CCW order
    vec4f* N; // Vert Normals: _____ Nvrt X {x,y,z}, Normal is the "Zdir" of local coord system
    vec3u* A; // Adjacent Triangles: Ntri X {f1,f2,f3}, CCW order
}TriNet;


///// Vertex Array Object (VAO, Nested) ///////////////////////////////////

typedef struct{
    // Vertex Array Object meant to be drawn rapidly and simply

    /// Geo Info ///
    uint   Ntri; //- Number of triangles
    float* V; // --- `Ntri` * 9: `float`
    float* N; // --- `Ntri` * 9: `float`
    float* C; // --- `Ntri` * 9: `float`
    uint   bufID; // Buffer ID at the GPU
    uint   arSiz; // Array size in bytes
    
    /// Pose & Scale ///
    float* relPose; // Static offset pose from parent pose (anchor)
    float* ownPose; // Dynamic offset pose from `relPose`
    vec4f  scale; // - Scale in each dimension
    float* totPose; // Total pose relative to parent frame, Accounting for {`relPose`, `ownPose`, `scale`}
    
    /// Composite VAO ///
    uint   Nprt; //- Number of sub-parts
    void** parts; // Array of sub-part pointers

}VAO_VNC_f;



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// transform.c ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// HOMOGENEOUS COORDINATES /////////////////////////////////////////////////////////////////

void  set_posn_mtx44f( float mat[], const vec4f posn ); // Set the position components of the homogeneous coordinates
vec4f mult_mtx44f_vec4f( const float mat[], const vec4f v ); // Transform `v` with `mat`
    


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// TriNet.c ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

////////// POLYHEDRAL NET //////////////////////////////////////////////////////////////////////////

///// Construction & Queries //////////////////////////////////////////////

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
// Return the average edge length in the polyhedron, Useful for getting the "scale" of the faces
float avg_edge_len( TriNet* net );

///// Rendering ///////////////////////////////////////////////////////////
    
void draw_net_wireframe( TriNet* net, vec4f lineColor ); // Draw the net as a wireframe, NOTE: Only `V` and `F` data req'd
void draw_net_connectivity( TriNet* net, vec4f lineColor ); // Draw the net neighbors as a wireframe


////////// SPECIFIC POLYHEDRA //////////////////////////////////////////////////////////////////////

///// Tetrahedron /////////////////////////////////////////////////////////
// Load geometry for an tetrahedron onto matrices `V` and `F` 
void populate_tetra_vertices_and_faces( vec4f* V, vec3u* F, float radius );
TriNet* create_tetra_mesh_only( float radius ); // Create an regular tetrahedron (*without* unfolded net data)

///// Cube ////////////////////////////////////////////////////////////////
// Load geometry for a cube onto matrices `V` and `F` 
void populate_cube_vertices_and_faces( vec4f* V, vec3u* F, float sideLen );
TriNet* create_cube_mesh_only( float sideLen ); // Create an regular icosahedron (*without* unfolded net data)

///// Icosahedron /////////////////////////////////////////////////////////
// Load geometry for an icosahedron onto matrices `V` and `F` 
void populate_icos_vertices_and_faces( vec4f* V, vec3u* F, float radius );
TriNet* create_icos_mesh_only( float radius ); // Create an regular icosahedron (*without* unfolded net data)
TriNet* create_icos_VFNA( float radius ); // Create an regular icosahedron (*with* unfolded net data)

///// Sphere from Divided Icos ////////////////////////////////////////////
// Construct a sphere with `radius` from a subdivided icos (`div` rows) and center at {0,0,0}
TriNet* create_icosphere_mesh_only( float radius, uint div );
TriNet* create_icosphere_VFNA( float radius, uint div ); // Create an regular icosahedron (*with* unfolded net data)



////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// VAO_VNC_f.c ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
    
////////// VERTEX ARRAY OBJECTS (NESTED) ///////////////////////////////////////////////////////////

VAO_VNC_f* make_VAO_VNC_f( uint Ntri_ ); // Allocate the VAO at heap
void /*-*/ allocate_N_VAO_VNC_parts( VAO_VNC_f* vao, uint N ); // Make space for `N` sub-part pointers
VAO_VNC_f* get_part_i( VAO_VNC_f* vao, uint i ); // Fetch sub-VAO
void /*-*/ delete_VAO_VNC_f( VAO_VNC_f* vao ); // Erase the VAO (and parts) at heap and the GPU
// Copy {V,N,C} from the specified arrays
void /*-*/ load_VAO_VNC_from_full_arrays( VAO_VNC_f* vao, /*<<*/ const float* Vsto, const float* Nsto, const float* Csto );
void /*-*/ allocate_and_load_VAO_VNC_at_GPU( VAO_VNC_f* vao ); // Fetch & set buffer ID, and make space on the GPU for the VAO
// Compose relative, ownship, and scale transformations into `totPose`, relative to parent frame
void /*-*/ update_total_pose( VAO_VNC_f* vao );
void /*-*/ draw_VAO_VNC_f( VAO_VNC_f* vao ); // Draw VAO and all subparts
vec4f /**/ get_posn( VAO_VNC_f* vao ); // Get the position components of the homogeneous coordinates as a vector
void /*-*/ set_posn( VAO_VNC_f* vao, const vec4f posn ); // Set the position components of the homogeneous coordinates
// Increment the position components of the homogeneous coordinates by the associated `delta` components
void /*-*/ translate( VAO_VNC_f* vao, const vec4f delta );
void /*-*/ rotate_angle_axis_rad( VAO_VNC_f* vao, float angle_rad, const vec4f axis ); // Rotate the object by `angle_rad` about `axis`
void /*-*/ rotate_RPY_vehicle( VAO_VNC_f* vao, float r_, float p_, float y_ ); // Increment the world Roll, Pitch, Yaw of the model
void /*-*/ thrust_Z_vehicle( VAO_VNC_f* vao, float dZ ); // Move in the local Z direction by `dZ` 

////////// CONSTRUCTION ////////////////////////////////////////////////////////////////////////////

VAO_VNC_f* VAO_from_TriNet_solid_color( TriNet* net, const vec4f color ); // Get a VAO from a `TriNet`
// Get a VAO from a `TriNet` with `xfrm` applied to all vertices and normals
VAO_VNC_f* VAO_from_TriNet_solid_color_transformed( TriNet* net, const vec4f color, const float* xfrm );

////////// SPECIFIC VAO ////////////////////////////////////////////////////////////////////////////

///// Cube ////////////////////////////////////////////////////////////////
// Construct a cube VAO with flat-shaded normals and one solid color
VAO_VNC_f* cube_VAO_VNC_f( float sideLen, const vec4f color );
VAO_VNC_f* colorspace_cube_VAO_VNC_f( void ); // Make a colorful cube from the static array data

///// Tetrahedron /////////////////////////////////////////////////////////
// Construct a tetrahedron VAO with flat-shaded normals and one solid color
VAO_VNC_f* tetrahedron_VAO_VNC_f( float radius, const vec4f color ); 
// Construct a tetrahedron VAO with flat-shaded normals and one solid color, with all vectors transformed
VAO_VNC_f* tetrahedron_transformed_VAO_VNC_f( float radius, const vec4f color, const float* xfrm );

///// Icosahedron /////////////////////////////////////////////////////////
// Construct a icosahedron VAO with flat-shaded normals and one solid color
VAO_VNC_f* icosahedron_VAO_VNC_f( float radius, const vec4f color );

///// Icosphere ///////////////////////////////////////////////////////////
// Construct a icosphere VAO with flat-shaded normals and one solid color
VAO_VNC_f* icosphere_VAO_VNC_f( float radius, uint div, const vec4f color );
    


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// draw_geo.c /////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
    
void draw_sphere( vec4f center, float radius, vec4f color ); // Draws a sphere divided on lat/long
// Draw a square grid centered at the origin, extending 'xPlusMinus' units in X and 'yPlusMinus' units in Y
void draw_grid_org_XY( float gridSize, uint xPlusMinus, uint yPlusMinus, float lineThic, vec4f color );
    

#endif
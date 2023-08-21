// g++ 18_shiny-ribbons.cpp -std=c++17 -lraylib
// Re-implement Boid Ribbons **without** `Model`s!
// 2023-08-14: Do not break into smaller files until everything works

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <iostream>
using std::cout, std::endl;
#include <stdlib.h>  // srand, rand
#include <time.h>
#include <array>
using std::array;
#include <vector>
using std::vector;
#include <algorithm>
using std::clamp, std::min;
#include <memory>
using std::shared_ptr;
#include <deque>
using std::deque;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

/// Local ///
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"


///// Aliases ////////////////////////////////////
typedef unsigned char /*---*/ ubyte;
typedef unsigned long /*---*/ ulong;
typedef vector<float> /*---*/ vf;
typedef vector<vector<float>> vvf;
typedef vector<Vector3> /*-*/ vvec3;
typedef array<Vector3,3> /**/ triPnts; // Vector info for One Triangle (Vertices,nrms) 
typedef array<Color,3> /*--*/ triClrs; // Color  info for One Triangle
typedef vector<Vector3> /*-*/ vvec3; // - Vector of 3D vectors
#define VERTEX_BUFFER_IDX 0 // Vertex coord VBO
#define NORMAL_BUFFER_IDX 2 // Normal vector VBO
#define COLORS_BUFFER_IDX 3 // Vertex color VBO
#define INDEXF_BUFFER_IDX 6 // Indices of facet vertices VBO


////////// RANDOM NUMBERS //////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    // NOTE: This function assumes `hi > lo`
    float span = hi - lo;
    return lo + span * randf();
}

int randi( int lo, int hi ){
    // Return a pseudo-random number between `lo` and `hi` (int)
    int span = hi - lo;
    return lo + (rand() % span);
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed



////////// VECTOR OPERATIONS ///////////////////////////////////////////////////////////////////////


Vector3 normal_of_tiangle( const triPnts& tri ){
    // Get the normal of the triangle assuming CCW / Right-Hand ordering
    return Vector3Normalize( Vector3CrossProduct(
        Vector3Subtract( tri[2], tri[1] ),
        Vector3Subtract( tri[0], tri[1] )
    ));
}

Vector3 uniform_vector_noise( const Vector3& vec, float halfMag ){
    // Return a perturbed `vec` moved up to `halfMag` in each direction in each dimension
    return Vector3{
        randf( vec.x - halfMag, vec.x + halfMag ),
        randf( vec.y - halfMag, vec.y + halfMag ),
        randf( vec.z - halfMag, vec.z + halfMag )
    };
}

Vector3 uniform_vector_noise( float halfMag ){
    // Return a perturbed `vec` moved up to `halfMag` in each direction in each dimension
    return Vector3{
        randf( -halfMag, halfMag ),
        randf( -halfMag, halfMag ),
        randf( -halfMag, halfMag )
    };
}

////////// VECTOR MATH STRUCTS /////////////////////////////////////////////////////////////////////

struct Basis{
    // Pose exchange format for `Boid`s, Z-basis has primacy
    // NOTE: None of the operations with other bases assume any operand is orthonormalized
    // NOTE: Position is largely absent from Basis-Basis operations

    /// Members ///
    Vector3 Xb; // X-basis
    Vector3 Yb; // Y-basis
    Vector3 Zb; // Z-basis
    Vector3 Pt; // Position

    /// Static Methods ///

    static Basis origin(){
        // Get the origin `Basis`
        Basis rtnBasis;
        rtnBasis.Xb = Vector3{ 1.0f, 0.0f, 0.0f };
        rtnBasis.Yb = Vector3{ 0.0f, 1.0f, 0.0f };
        rtnBasis.Zb = Vector3{ 0.0f, 0.0f, 1.0f };
        rtnBasis.Pt = Vector3{ 0.0f, 0.0f, 0.0f };
        return rtnBasis;
    }

    static Basis random(){
        // Sample from a +/-1.0f cube for all vectors, then `orthonormalize`
        Basis rtnBasis;
        rtnBasis.Xb = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.Yb = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.Zb = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.Pt = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.orthonormalize();
        return rtnBasis;
    }

    static Basis from_transform_and_point( const Matrix& xform, const Vector3& point ){
        // Get a `Basis` from a `xform` matrix and a `point`
        Basis rtnBasis;
        rtnBasis.Xb = Vector3Transform( Vector3{1.0, 0.0, 0.0}, xform );
        rtnBasis.Yb = Vector3Transform( Vector3{0.0, 1.0, 0.0}, xform );
        rtnBasis.Zb = Vector3Transform( Vector3{0.0, 0.0, 1.0}, xform );
        rtnBasis.Pt = point;
        return rtnBasis;
    }

    /// Methods ///

    void orthonormalize(){
        // Make sure this is an orthonormal basis, Z-basis has primacy
        // No need to normalize `Xb`, see below
        Yb = Vector3Normalize( Yb );
        Zb = Vector3Normalize( Zb );
        Xb = Vector3Normalize( Vector3CrossProduct( Yb, Zb ) );
        Yb = Vector3Normalize( Vector3CrossProduct( Zb, Xb ) );
    };

    void blend_orientations_with_factor( const Basis& other, float factor ){
        // Exponential filter between this basis and another Orthonormalize separately
        // 1. Clamp factor
        factor = clamp( factor, 0.0f, 1.0f );
        // 2. Blend bases
        // No need to compute `Xb`, see `orthonormalize`
        Yb = Vector3Add(  Vector3Scale( Yb, 1.0-factor ), Vector3Scale( other.Yb, factor )  );
        Zb = Vector3Add(  Vector3Scale( Zb, 1.0-factor ), Vector3Scale( other.Zb, factor )  );
        // 3. Correct basis
        orthonormalize();
    }

    Basis operator+( const Basis& other ){
        // Addition operator for bases, Just the vector sum of each basis, Orthonormalize separately
        Basis rtnBasis;
        rtnBasis.Xb = Vector3Add( Xb, other.Xb );
        rtnBasis.Yb = Vector3Add( Yb, other.Yb );
        rtnBasis.Zb = Vector3Add( Zb, other.Zb );
        rtnBasis.Pt = Vector3Add( Pt, other.Pt );
        return rtnBasis;
    }

    Basis get_scaled_orientation( float factor ){
        // Return a copy of the `Basis` with each individual basis scaled by a factor
        Basis rtnBasis;
        rtnBasis.Xb = Vector3Scale( Xb, factor );
        rtnBasis.Yb = Vector3Scale( Yb, factor );
        rtnBasis.Zb = Vector3Scale( Zb, factor );
        // rtnBasis.Pt = Vector3Scale( Pt, factor );
        return rtnBasis;
    }

    Basis copy(){
        // Return a copy of the basis.
        Basis rtnBasis;
        rtnBasis.Xb = Vector3( Xb );
        rtnBasis.Yb = Vector3( Yb );
        rtnBasis.Zb = Vector3( Zb );
        rtnBasis.Pt = Vector3( Pt );
        return rtnBasis;
    }
};
typedef shared_ptr<Basis> basPtr; // 2023-06-17: For now assume that Bases are lightweight enough to pass by value



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////


class DynaMesh{ public:
    // Straightforward container for a `Mesh` with changing geometry
    // 2023-08-14: This class assumes that the mesh will have a constant size for each buffer, though their values might change
    // 2023-08-15: This class REQUIRES that vertices, indices, normals, and colors ALL be defined

    /// Members ///
    vector<triPnts> tris; // Dynamic geometry, each array is a facet of 3x vertices
    vector<triPnts> nrms; // Normal vector for each facet
    vector<triClrs> clrs; // Vertex colors for each facet
    Mesh /*------*/ mesh; // Raylib mesh geometry
    bool /*------*/ upld; // Has the mesh been uploaded?
    Matrix /*----*/ xfrm; // Pose in the parent frame
    Model /*-----*/ modl; // Needed for shaders
    Shader /*----*/ shdr; // Needed for shaders
    Color /*-----*/ bClr; // Base color
    
    /// Memory Methods ///

    void init_mesh_memory( uint Ntri ){
        // Create the `Mesh` struct and allocate memory there

        uint Npts = Ntri*3;

        // 1. Init mesh
        upld = false;
        mesh = Mesh{};
        mesh.triangleCount = Ntri;
        mesh.vertexCount   = Npts;
        
        // 3. Init memory
        mesh.vertices = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices  = (ushort*) MemAlloc(Npts   * sizeof( ushort ));
        mesh.normals  = (float* ) MemAlloc(Npts*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.colors   = (u_char*) MemAlloc(Npts*4 * sizeof( u_char )); // 3 vertices, 4 coordinates each (r, g, b, a)
    }

    void load_mesh_buffers( bool loadGeo = true, bool loadColor = false ){
        // Load geometry (and color) info into `mesh` buffers        
        // 2023-08-14: For now assume that the facet indices do NOT change!

        // 0. Init
        ulong k = 0; // Vertex _ counter
        ulong l = 0; // Index __ counter
        ulong m = 0; // Color __ counter

        // 1. Load CPU-side
        for( uint i = 0; i < mesh.triangleCount; ++i ){
            for( u_char j = 0; j < 3; j++ ){
                if( loadGeo ){
                    mesh.normals[k]  = nrms[i][j].x;
                    mesh.vertices[k] = tris[i][j].x;  k++;
                    mesh.normals[k]  = nrms[i][j].y;
                    mesh.vertices[k] = tris[i][j].y;  k++;
                    mesh.normals[k]  = nrms[i][j].z;
                    mesh.vertices[k] = tris[i][j].z;  k++;
                    mesh.indices[l]  = l; /*-------*/ l++; 
                }
                if( loadColor ){
                    mesh.colors[m] = clrs[i][j].r;  m++;
                    mesh.colors[m] = clrs[i][j].g;  m++;
                    mesh.colors[m] = clrs[i][j].b;  m++;
                    mesh.colors[m] = clrs[i][j].a;  m++;
                }
            }
        }

        // If this mesh is not present on the GPU, then send it
        if( !upld ){
            // 3. Initial load to GPU
            UploadMesh( &mesh, true );
            upld = true;
        // Else mesh is on GPU, update the mesh buffers there
        }else{
            // 2. Send GPU-side
            if( loadGeo ){
                UpdateMeshBuffer( 
                    mesh, // -------------------------------- `Mesh` object
                    VERTEX_BUFFER_IDX, // ------------------- VBO Index
                    mesh.vertices, // ----------------------- Array of data 
                    mesh.vertexCount*3 * sizeof( float  ), // Total array size
                    0 // ------------------------------------ Starting index in array
                );
                UpdateMeshBuffer( 
                    mesh, // -------------------------------- `Mesh` object
                    NORMAL_BUFFER_IDX, // ------------------- VBO Index
                    mesh.normals, // ------------------------ Array of data 
                    mesh.vertexCount*3 * sizeof( float  ), // Total array size
                    0 // ------------------------------------ Starting index in array
                );
                // 2023-08-14: For now assume that the facet indices do NOT change!
            }
            if( loadColor ){
                UpdateMeshBuffer( 
                    mesh, // -------------------------------- `Mesh` object
                    COLORS_BUFFER_IDX, // ------------------- VBO Index
                    mesh.colors, // ------------------------- Array of data 
                    mesh.vertexCount*4 * sizeof( u_char ), // Total array size
                    0 // ------------------------------------ Starting index in array
                );
            }
        }
    }

    void remodel(){
        // Reset the `Model`
        modl = LoadModelFromMesh( mesh );
        modl.materials[0].shader = shdr;
    }

    /// Constructors ///

    DynaMesh( uint Ntri ){
        // Allocate memory and set default pose
        init_mesh_memory( Ntri );
        xfrm = MatrixIdentity();
        bClr = WHITE;
    }

    /// Geometry Methods ///

    void push_triangle_w_norms( triPnts tri ){
        // Add one triangle
        Vector3 norm = normal_of_tiangle( tri );  
        tris.push_back( tri );
        nrms.push_back({ norm, norm, norm });
    }

    /// Pose Math ///
    /* xfrm = 
    m0 m4 m8  m12
    m1 m5 m9  m13
    m2 m6 m10 m14
    m3 m7 m11 m15
    */

    Vector3 get_posn(){
        // Get the position components of the homogeneous coordinates as a vector
        return Vector3{ xfrm.m12, xfrm.m13, xfrm.m14 };
    }

    void set_posn( const Vector3& posn ){
        // Set the position components of the homogeneous coordinates
        xfrm.m12 = posn.x;
        xfrm.m13 = posn.y;
        xfrm.m14 = posn.z;
    }

    void translate( const Vector3& delta ){
        // Increment the position components of the homogeneous coordinates by the associated `delta` components
        xfrm.m12 += delta.x;
        xfrm.m13 += delta.y;
        xfrm.m14 += delta.z;
    }

    void rotate_RPY( float r_, float p_, float y_ ){
		// Increment the world Roll, Pitch, Yaw of the model
        xfrm = MatrixMultiply( 
            MatrixMultiply( MatrixMultiply( MatrixRotateY( y_ ), MatrixRotateX( p_ ) ), MatrixRotateZ( r_ ) ), 
            xfrm 
        );
	}

    ///// Drawing Context Methods ////////////////
    // NOTE: Method beyond this point require a drawing context being instantiated before calling!

    /// Rendering ///

    void set_shader( Shader shader ){ shdr = shader; }

    void draw(){
        // Render the mesh
        modl.transform = xfrm;
        DrawModel( modl, get_posn(), 1.0f, bClr );
    }

};

class FractureCube : public DynaMesh{ public:
    // A cube that shakes apart

    /// Constructors ///

    FractureCube( float sideLen ) : DynaMesh( 12 ){
        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm;
        vvec3   V;
        float   halfLen = sideLen/2.0;
        Color   R{ 255,   0,   0, 255 };
        Color   G{   0, 255,   0, 255 };
        Color   B{   0,   0, 255, 255 };

        // 1. Establish vertices
        V.push_back( Vector3{ -halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen,  halfLen } );

        // 2. Build tris
        push_triangle_w_norms( { V[0], V[3], V[2] } );
        push_triangle_w_norms( { V[0], V[1], V[3] } );
        push_triangle_w_norms( { V[6], V[4], V[0] } );
        push_triangle_w_norms( { V[6], V[0], V[2] } );
        push_triangle_w_norms( { V[0], V[4], V[5] } );
        push_triangle_w_norms( { V[0], V[5], V[1] } );
        push_triangle_w_norms( { V[7], V[6], V[2] } );
        push_triangle_w_norms( { V[7], V[2], V[3] } );
        push_triangle_w_norms( { V[4], V[6], V[7] } );
        push_triangle_w_norms( { V[4], V[7], V[5] } ); 
        push_triangle_w_norms( { V[1], V[5], V[7] } );
        push_triangle_w_norms( { V[1], V[7], V[3] } );

        // 3. Build Colors
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );
        pushClr = { R, G, B };  clrs.push_back( pushClr );
        pushClr = { B, R, G };  clrs.push_back( pushClr );
        pushClr = { G, B, R };  clrs.push_back( pushClr );

        // 4. Load buffers
        load_mesh_buffers( true, true );
    }

    void update(){
        // translate( uniform_vector_noise( 0.125 ) );
        if( randf() < 0.50 ){
            for( triPnts& tri : tris ){
                tri[0] = uniform_vector_noise( tri[0], 0.125 );
                tri[1] = uniform_vector_noise( tri[1], 0.125 );
                tri[2] = uniform_vector_noise( tri[2], 0.125 );
                // 2023-08-13: Purposely avoiding a normal update until a shader is in use
            }
        }
        load_mesh_buffers( true, false );
        remodel();
    }
};

struct Sphere{
    // Container struct for an obstacle to avoid
    
    /// Members ///
    Vector3 cntr;
    float   rads;
    Mesh    mesh; // Raylib mesh geometry
    bool    upld; // Has the mesh been uploaded?
    Matrix  xfrm; // Pose in the parent frame
    Model   modl; // Needed for shaders
    Shader  shdr; // Needed for shaders
    Color   colr;

    void set_posn( const Vector3& posn ){
        // Set the position components of the homogeneous coordinates
        xfrm.m12 = posn.x;
        xfrm.m13 = posn.y;
        xfrm.m14 = posn.z;
    }

    void translate( const Vector3& delta ){
        // Increment the position components of the homogeneous coordinates by the associated `delta` components
        xfrm.m12 += delta.x;
        xfrm.m13 += delta.y;
        xfrm.m14 += delta.z;
    }

    /// Constructors ///
    
    Sphere(){
        cntr = Vector3Zero();
        rads = 1.0;
        mesh = GenMeshSphere( rads, 32, 32 );
        upld = false;
        xfrm = MatrixIdentity();
        colr = GRAY;
        set_posn( cntr );
    }

    Sphere( const Vector3& center, float radius, Color color = GRAY ){
        cntr = center;
        rads = radius;
        mesh = GenMeshSphere( rads, 32, 32 );
        upld = false;
        xfrm = MatrixIdentity();
        colr = color;
        set_posn( cntr );
    }

    /// Methods ///

    void remodel(){
        // Reset the `Model`
        modl = LoadModelFromMesh( mesh );
        modl.materials[0].shader = shdr;
    }

    /// Rendering ///

    void set_shader( Shader shader ){ shdr = shader; }

    void draw(){
        // Render the mesh
        modl.transform = xfrm;
        DrawModel( modl, Vector3Zero(), 1.0f, colr );
    }

    Sphere copy() const {  return Sphere{ cntr, rads };  }
};

uint Nboids = 0;

class BoidRibbon : public DynaMesh{ public:

    /// Members ///
    uint  ID; // --- Identifier
    
    /// Pose ///
    Basis headingB; // Where the boid is actually pointed

    /// Way-Finding ///
    float   ur; // ----- Update rate
    float   dNear; // -- Radius of hemisphere for flocking consideration
    uint    Nnear; // -- How many neighbors are there?
    float   scale; // -- Habitat scale
    Vector3 home; // --- Don't get too far from this point
    Basis   flocking; // Flocking instinct
    Basis   homeSeek; // Home seeking instinct
    Basis   freeWill; // Drunken walk
    Basis   avoidSph; // Sphere avoidance instinct
    Sphere  fearSphr; // The sphere to fear

    /// Rendering ///
    uint /*--------------*/ Npairs; // -- Number of coordinate pairs allowed
    uint /*--------------*/ Nviz; // ---- Number of coordinate pairs visible now
    float /*-------------*/ headAlpha; // Beginning opacity
    float /*-------------*/ tailAlpha; // Ending    opacity
    deque<array<Vector3,2>> coords; // -- Ribbon data, listed from head to tail
    double /*------------*/ width; // --- Width of the ribbon 

    /// Constructor ///

    BoidRibbon( uint N_pairs, float width_, float d_Near, float updateRate, float alphaHead = 1.0f, float alphaTail = 0.0f,
                const Vector3& home_ )
        : DynaMesh( (N_pairs-1)*4 ){
        // Build the geometry of the boid
        Npairs    = N_pairs;
        headAlpha = alphaHead;
        tailAlpha = alphaTail;
        width     = width_;
        dNear     = d_Near;
        ur /*--*/ = updateRate;
        home /**/ = home_;
        bClr /**/ = Color{
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            255
        };
        headingB = Basis::random(); // Where the boid is actually pointed
        flocking = headingB.copy(); // Flocking instinct
        homeSeek = headingB.copy(); // Home seeking instinct
        freeWill = headingB.copy(); // Drunken walk
        avoidSph = headingB.copy(); // Sphere avoidance instinct

        Nboids++;
        ID = Nboids;
    }

    /// Methods ///

    Basis get_Basis(){
        // Get pose info for this boid
        return headingB.copy();
    }

    void push_coord_pair( const Vector3& c1, const Vector3& c2 ){
        // Add coordinates to the head of the plume
        if( coords.size() >= Npairs )  coords.pop_back(); // If queue is full, drop the tail element
        coords.push_front(  array<Vector3,2>{ c1, c2 }  );
    }

    /// Navigation ///

    Basis consider_neighbors( const vector<Basis>& nghbrPoses ){
        // Flocking instinct: Adjust bearing according to the closest neighbors in front of the boid
        Vector3 Xmean;
        Vector3 Ymean;
        Vector3 Zmean;
        Vector3 Zfly , diffVec;
        Basis   rtnMsg;
        uint    relevant = 0;
        float   dist, dotFront;
        Xmean = Vector3{0.0f, 0.0f, 0.0f};
        Ymean = Vector3{0.0f, 0.0f, 0.0f};
        Zmean = Vector3{0.0f, 0.0f, 0.0f};
        for( const Basis& msg : nghbrPoses ){
            diffVec  = Vector3Subtract( msg.Pt, headingB.Pt );
            dist     = Vector3Length( diffVec );
            if( dist > 0.0 ){
                Zfly     = headingB.Zb;
                dotFront = Vector3DotProduct( Zfly, diffVec );
                if( (dist <= dNear) || (dotFront >= 0.0f) ){
                    Xmean  = Vector3Add( Xmean, msg.Xb );
                    Ymean  = Vector3Add( Ymean, msg.Yb );
                    Zmean  = Vector3Add( Zmean, msg.Zb );
                    relevant++;
                }
            }
        }
        Nnear = relevant;
        if( relevant ){
            rtnMsg.Xb = Xmean;
            rtnMsg.Yb = Ymean;
            rtnMsg.Zb = Zmean;
            rtnMsg.orthonormalize();
        }else{
            rtnMsg.Xb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Yb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Zb = Vector3{0.0f, 0.0f, 0.0f};
        }
        return rtnMsg;
    }

    Basis consider_home(){
        // Home seeking instinct: Take `N_samples` and choose the one that points closest to `home`
        Basis   rtnMsg;
        Vector3 hVec = Vector3Subtract( home, headingB.Pt );
        rtnMsg.Zb = Vector3Normalize( hVec );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( headingB.Yb, rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb  , rtnMsg.Xb ) );
        return rtnMsg;
    }

    Basis consider_free_will(){
        // Drunken walk: Choose a random direction in which to nudge free will
        Basis   rtnMsg;
        Vector3 vWil = Vector3{
            randf( -1.0,  1.0 ),
            randf( -1.0,  1.0 ),
            randf( -1.0,  1.0 )
        };
        rtnMsg.Zb = Vector3Normalize( vWil );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( headingB.Yb, rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb  , rtnMsg.Xb ) );
        return rtnMsg;
    }

    Basis consider_spheres( const vector<Sphere>& sphereList ){
        // Sphere avoidance instinct
        float nearDist = 1e9;
        float sphrDist = 0.0;
        uint  i /*--*/ = 0;
        // 1. Find the closest sphere
        for( const Sphere& sphere : sphereList ){
            sphrDist = Vector3Distance( sphere.cntr, headingB.Pt );
            if( sphrDist < nearDist ){
                nearDist = sphrDist;
                fearSphr = sphere.copy();
            }
            i++;
        }
        // 2. Generate an avoiding heading
        Vector3 toSphr = Vector3Subtract( fearSphr.cntr, headingB.Pt );
        if( nearDist < fearSphr.rads ){
            Basis rtnBasis;
            rtnBasis.Zb = Vector3Scale( toSphr, -1.0f );
            rtnBasis.Yb = Vector3CrossProduct( rtnBasis.Zb, headingB.Xb );
            rtnBasis.orthonormalize();
            return rtnBasis;
        }else if( Vector3DotProduct( headingB.Zb, toSphr ) > 0.0 ){
            Basis rtnBasis;
            rtnBasis.Yb = Vector3Scale( toSphr, -1.0 );
            rtnBasis.Xb = Vector3CrossProduct( rtnBasis.Yb, headingB.Zb );
            rtnBasis.Zb = Vector3CrossProduct( rtnBasis.Xb, rtnBasis.Yb );
            rtnBasis.orthonormalize();
            return rtnBasis;
        }else{
            return headingB.copy();
        }
    }

    double update_instincts_and_heading( const vector<Basis>& flockPoses, const vector<Sphere>& spheres ){
        // Main navigation function
        // 1. Update flocking instinct
        Basis flockDrive = consider_neighbors( flockPoses );
        flocking.blend_orientations_with_factor( flockDrive, ur );
        // 2. Update home seeking instinct
        float dist /*-*/ = Vector3Distance( home, headingB.Pt );
        Basis centrDrive = consider_home();
        homeSeek.blend_orientations_with_factor( centrDrive, ur );
        // 3. Update drunken walk
        if( randf() < 0.25 ){
            Basis freewDrive = consider_free_will();
            freeWill.blend_orientations_with_factor( freewDrive, ur );
        }
        // 4. Update sphere avoidance instinct
        Basis fearDrive = consider_spheres( spheres );
        avoidSph.blend_orientations_with_factor( fearDrive, ur );
        float fearDist = Vector3Distance( fearSphr.cntr, headingB.Pt );
        // 4. Blend intincts
        Basis total = flocking.get_scaled_orientation( 0.45f * Nnear ) + 
                      homeSeek.get_scaled_orientation( dist/scale*5.0f ) + 
                      freeWill.get_scaled_orientation( 10.0 ) + 
                      avoidSph.get_scaled_orientation( fearSphr.rads / fearDist * 15 );
        // 5. Limit turn and set heading
        double updateTurn = Vector3Angle( total.Zb, headingB.Zb );
        double turnMax    = PI/32;
        double factor     = updateTurn/turnMax;

        headingB.blend_orientations_with_factor( total, ur/factor );
        return 0.0;
    }

    void update_position( float zThrust ){
        // Move forward and push a segment to the ribbon
        // 1. Z Thrust
        headingB.Pt = Vector3Add( headingB.Pt, Vector3Scale( headingB.Zb, zThrust ) );
        push_coord_pair(
            Vector3Add( headingB.Pt, Vector3Scale( headingB.Xb,  width/2.0f ) ),
            Vector3Add( headingB.Pt, Vector3Scale( headingB.Xb, -width/2.0f ) )
        );
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void update(){
        // Create geometry
        /*---*/ Nviz  = coords.size()-1;        
        float   R     = bClr.r/255.0f;
        float   G     = bClr.g/255.0f;
        float   B     = bClr.b/255.0f;
        float   Aspan = headAlpha - tailAlpha;
        Vector3 c1, c2, c3, c4, n1, n2;
        float   A_i;
        float   A_ip1;

        for( uint i = 0; i < Nviz; i++){
            c1    = coords[i  ][0];
            c2    = coords[i  ][1];
            c3    = coords[i+1][0];
            c4    = coords[i+1][1];
            A_i   = tailAlpha + Aspan*(Nviz-(i  ))/(1.0f*Nviz);
            A_ip1 = tailAlpha + Aspan*(Nviz-(i+1))/(1.0f*Nviz);

            // FIXME, START HERE: PUSH TRIANGLES!

        }
    }

    void draw(){
        // Render plume as a `Model`
        

    }

};
typedef shared_ptr<BoidRibbon> rbbnPtr;

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Dynamic Box!" );
    SetTargetFPS( 60 );
    // rlEnableSmoothLines();
    // rlDisableBackfaceCulling();

    float halfBoxLen = 100.0/10.0;

    /// Init Objects ///
    FractureCube dc{ 5.0 };
    Sphere /*-*/ sp{ Vector3Zero(), 6.0, BLUE };

    // Camera
    Camera camera = Camera{
        Vector3{  15.0,  15.0,  15.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    // Load basic lighting shader
    Shader shader = LoadShader(TextFormat("shaders/lighting.vs"),
                               TextFormat("shaders/lighting.fs"));
    // Get some required shader locations
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    
    
    // Ambient light level (some basic lighting)
    float ambientColor[4] = { 0.25f, 0.25f, 0.25f, 0.50f };
    int   ambientLoc /**/ = GetShaderLocation(shader, "ambient");
    SetShaderValue( shader, ambientLoc, ambientColor, SHADER_UNIFORM_VEC4 );

    // Using just 1 point lights
    Light light = CreateLight(
        LIGHT_POINT, 
        Vector3{ 20.0, 20.0, 20.0 }, 
        Vector3Zero(), 
        Color{ 255, 255, 255, 125 }, 
        shader
    );

    dc.set_shader( shader );
    sp.set_shader( shader );
    sp.remodel();

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        // dc.update();
        // dc.remodel();

        UpdateLightValues( shader, light );

        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &camera.position.x, SHADER_UNIFORM_VEC3);

        ///// DRAW LOOP ///////////////////////////////////////////////////
        sp.translate( uniform_vector_noise( 0.125 ) );
        sp.draw();
        

        /// End Drawing ///
        EndMode3D();

        DrawFPS( 30, 30 );

        EndDrawing();
    }

    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////
    return 0;
}
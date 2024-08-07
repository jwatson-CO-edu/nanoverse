#ifndef RL_TOYBOX_H
#define RL_TOYBOX_H

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes ///////////////////////////////////

/// Standard ///
#include <array>
using std::array;
#include <vector>
using std::vector;
#include <memory>
using std::shared_ptr;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

/// Local ///
#include "utils.hpp"

///// Aliases ////////////////////////////////////
typedef array<Vector3,3> triPnts; // Vector info for One Triangle (Vertices,nrms) 
typedef array<Color,3>   triClrs; // Color  info for One Triangle
typedef vector<Vector3>  vvec3; // - Vector of 3D vectors
typedef array<Vector3,2> segment; // Vector info for One Line Segment 
#define VERTEX_BUFFER_IDX 0 // Vertex coord VBO
#define NORMAL_BUFFER_IDX 2 // Normal vector VBO
#define COLORS_BUFFER_IDX 3 // Vertex color VBO
#define INDEXF_BUFFER_IDX 6 // Indices of facet vertices VBO



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

Color uniform_random_color(){
    // Return a random fully opaque color
    return Color{ rand_ubyte(), rand_ubyte(), rand_ubyte(), 255 };
}

float Vector2CrossProduct( const Vector2& op1, const Vector2& op2 ){
    // Return the magnitude of the cross product of two 2D vectors
    return ( op1.x * op2.y - op1.y * op2.x );
}

Vector3 Vector3Error(){  return Vector3{ nanf(""), nanf(""), nanf("") };  } // Return a vector signifying an error

Matrix MatrixError(){
    // Return a `Matrix` signifying an error
    Matrix rtnMatx = MatrixIdentity();
    rtnMatx.m0  = nanf("");
    rtnMatx.m5  = nanf("");
    rtnMatx.m10 = nanf("");
    rtnMatx.m15 = nanf("");
    return rtnMatx;
}

bool p_vec3_err( const Vector3& q ){  return ( isnanf( q.x ) || isnanf( q.y ) || isnanf( q.z ) );  } // Return true for any elem NaN

Vector3 vec3d_from_arbitrary_2D_basis( float x, float y, const Vector3& xBasis, const Vector3& yBasis ){
    // Return a coordinate in an arbitrary (non-orthoginal) 2D basis nested within a 3D frame
    // DO NOT normalize the basis vectors , see below!
	return Vector3Add(  Vector3Scale( xBasis, x ),  Vector3Scale( yBasis, y )  ); 
}

triPnts flip_tri_outward( const triPnts& tri ){
    // Point the triangle normal away from the origin
    if( Vector3DotProduct( normal_of_tiangle( tri ), tri[0] ) > 0.0f )  return tri;
    return {tri[0], tri[2], tri[1]};
}


    

Matrix MatrixRotateAxisAngle( const Vector3& axis, float angle_rad ){
    // Return a homogeneous transform that is a rotation by `angle_rad` about the `axis`
    Matrix rtnMtx = MatrixIdentity();
    /* m0 m4 m8  m12
       m1 m5 m9  m13
       m2 m6 m10 m14
       m3 m7 m11 m15 */
    // 1. Calc components
    Vector3 axis_ = Vector3Normalize( axis );
    float   k1    = axis_.x;
    float   k2    = axis_.y;
    float   k3    = axis_.z;
    float   vTh   = vsnf( angle_rad );
    float   cTh   = cosf( angle_rad );
    float   sTh   = sinf( angle_rad );
    // 2. X-basis
    rtnMtx.m0 = k1*k1*vTh + cTh;
    rtnMtx.m1 = k2*k1*vTh + k3*sTh;
    rtnMtx.m2 = k3*k1*vTh - k2*sTh;
    // 3. Y-basis
    rtnMtx.m4 = k1*k2*vTh - k3*sTh;
    rtnMtx.m5 = k2*k2*vTh + cTh;
    rtnMtx.m6 = k3*k2*vTh + k1*sTh;
    // 4. Z-basis
    rtnMtx.m8  = k1*k3*vTh + k2*sTh;
    rtnMtx.m9  = k2*k3*vTh - k1*sTh;
    rtnMtx.m10 = k3*k3*vTh + cTh;
    // N. Return
    return rtnMtx;
}

////////// HOMOGENEOUS COORDINATES /////////////////////////////////////////////////////////////////

Matrix set_posn( const Matrix& xfrm, const Vector3& posn ){
    // Set the position components of the homogeneous coordinates
    Matrix rtnMatx{ xfrm };
    rtnMatx.m12 = posn.x;
    rtnMatx.m13 = posn.y;
    rtnMatx.m14 = posn.z;
    return rtnMatx;
}

Vector3 get_posn( const Matrix& xfrm ){
    // Set the position components of the homogeneous coordinates
    return Vector3{ xfrm.m12, xfrm.m13, xfrm.m14 };
}

Matrix rotate_RPY_vehicle( const Matrix& xfrm, float r_, float p_, float y_ ){
    // Increment the world Roll, Pitch, Yaw of the model
    // NOTE: This is for airplanes that move forward in their own Z and have a wingspan across X
    return MatrixMultiply( 
        MatrixMultiply( MatrixMultiply( MatrixRotateY( y_ ), MatrixRotateX( p_ ) ), MatrixRotateZ( r_ ) ), 
        xfrm 
    );
}

Matrix translate( const Matrix& xfrm, const Vector3& delta ){
    // Increment the position components of the homogeneous coordinates by the associated `delta` components
    Matrix rtnMatx{ xfrm };
    rtnMatx.m12 += delta.x;
    rtnMatx.m13 += delta.y;
    rtnMatx.m14 += delta.z;
    return rtnMatx;
}

Matrix thrust_Z_vehicle( const Matrix& xfrm, float dZ ){
    // Move in the local Z direction by `dZ` 
    Matrix R = set_posn( xfrm, Vector3Zero() );
    return translate( xfrm, Vector3Scale( Vector3Transform( Vector3{0.0,0.0,1.0}, R ) , dZ ) );
}

Matrix move_X_vehicle( const Matrix& xfrm, float dX ){
    // Move in the local X direction by `dX` 
    Matrix R = set_posn( xfrm, Vector3Zero() );
    return translate( xfrm, Vector3Scale( Vector3Transform( Vector3{1.0,0.0,0.0}, R ) , dX ) );
}

Matrix translate_XY( const Matrix& xfrm, const Vector2& trns ){
    // Move the pose within the local XY plane
    float x = round_to_N_places( trns.x, 4 );
    float y = round_to_N_places( trns.y, 4 );
    Vector3 Xlocal = Vector3Transform( { 1.0f, 0.0f, 0.0f}, set_posn( xfrm, Vector3Zero() ) ); 
    Vector3 Ylocal = Vector3Transform( { 0.0f, 1.0f, 0.0f}, set_posn( xfrm, Vector3Zero() ) ); 
    // cout << "X Move: "
    return translate( xfrm, Vector3Add(
        Vector3Scale( Xlocal, x ),
        Vector3Scale( Ylocal, y )
    ) );
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

    static Basis from_Xb_and_Zb( const Vector3& xb_, const Vector3& zb_ ){
        // Get a basis at zero position from X-basis and Z-Basis
        Basis rtnBasis;
        rtnBasis.Yb = Vector3Normalize( Vector3CrossProduct( zb_, xb_ ) );
        rtnBasis.Zb = Vector3Normalize( zb_ );
        rtnBasis.orthonormalize();
        rtnBasis.Pt = Vector3Zero();
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
        // 2. Blend bases, No need to compute `Xb`, see `orthonormalize`
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

    /// Pose Math ///
    /* xfrm = 
    m0 m4 m8  m12
    m1 m5 m9  m13
    m2 m6 m10 m14
    m3 m7 m11 m15
    */

    Matrix get_homog(){
        Matrix rtnMatx = MatrixIdentity();
        // X-Basis
        rtnMatx.m0  = Xb.x;
        rtnMatx.m1  = Xb.y;
        rtnMatx.m2  = Xb.z;
        // Y-Basis
        rtnMatx.m4  = Yb.x;
        rtnMatx.m5  = Yb.y;
        rtnMatx.m6  = Yb.z;
        // Z-Basis
        rtnMatx.m8  = Zb.x;
        rtnMatx.m9  = Zb.y;
        rtnMatx.m10 = Zb.z;
        // Position
        rtnMatx.m12 = Pt.x;
        rtnMatx.m13 = Pt.y;
        rtnMatx.m14 = Pt.z;
        // Return
        return rtnMatx;
    }
};
typedef shared_ptr<Basis> basPtr; // 2023-06-17: For now assume that Bases are lightweight enough to pass by value



////////// BASIC TOYS //////////////////////////////////////////////////////////////////////////////

struct XY_Grid{
    // Simplest X-Y grid with regular spacing

    /// Members ///
    Vector3 cntr;
    float   xLen;
    float   yLen;
    float   unit;
    Color   colr;

    /// Construction Constants ///
    float xMin;
    float xMax;
    float yMin;
    float yMax;

    /// Constructor(s) ///

    XY_Grid( const Vector3& cntr_, float xLen_, float yLen_, float unit_, Color colr_ ){
        // Set vars for drawing
        cntr = cntr_;
        xLen = xLen_;
        yLen = yLen_;
        unit = unit_;
        colr = colr_;
        xMin = cntr.x - xLen/2.0f;
        xMax = cntr.x + xLen/2.0f;
        yMin = cntr.y - yLen/2.0f;
        yMax = cntr.y + yLen/2.0f;
    }

    /// Methods ///

    void draw(){
        // Draw the grid using lines
        float   X = unit;
        float   Y = unit;
        rlBegin( RL_LINES );

        rlColor4ub( colr.r, colr.g, colr.b, colr.a );

        rlVertex3f( cntr.x, yMin, cntr.z );
        rlVertex3f( cntr.x, yMax, cntr.z );

        while( (cntr.x + X) <= xMax ){
            rlVertex3f( cntr.x + X, yMin, cntr.z );
            rlVertex3f( cntr.x + X, yMax, cntr.z );
            rlVertex3f( cntr.x - X, yMin, cntr.z );
            rlVertex3f( cntr.x - X, yMax, cntr.z );
            X += unit;
        }

        rlVertex3f( xMin, cntr.y, cntr.z );
        rlVertex3f( xMax, cntr.y, cntr.z );

        while( (cntr.y + Y) <= yMax ){
            rlVertex3f( xMin, cntr.y + Y, cntr.z );
            rlVertex3f( xMax, cntr.y + Y, cntr.z );
            rlVertex3f( xMin, cntr.y - Y, cntr.z );
            rlVertex3f( xMax, cntr.y - Y, cntr.z );
            Y += unit;
        }

        rlEnd();
    }
};  



////////// MESH TOYS ///////////////////////////////////////////////////////////////////////////////

class DynaMesh{ public:
    // Straightforward container for a `Mesh` with changing geometry
    // 2023-08-14: This class assumes that the mesh will have a constant size for each buffer, though their values might change
    // 2023-08-15: This class REQUIRES that vertices, indices, normals, and colors ALL be defined

    /// Core Members ///
    uint /*------*/ Ntri; // --- Number of triangles
    uint /*------*/ Nvtx; // --- Number of vertices
    vector<triPnts> tris; // --- Dynamic geometry, each array is a facet of 3x vertices
    vector<triPnts> nrms; // --- Normal vector for each facet
    vector<triClrs> clrs; // --- Vertex colors for each facet
    Matrix /*----*/ xfrm; // --- Pose in the parent frame
    Matrix /*----*/ Trel; // --- Pose in the parent frame
    Matrix /*----*/ Tcur; // --- Pose in the parent frame
    Mesh /*------*/ mesh; // --- Raylib mesh geometry
    bool /*------*/ upldMesh; // Has the `Mesh` been uploaded?
    Model /*-----*/ modl; // --- Model
    bool /*------*/ upldModl; // Has the `Model` been uploaded?
    Shader /*----*/ shdr; // --- Shader
    bool /*------*/ sharVrtx; // Flag: Do the triangles share vertices?

    /// Optional Members ///
    vvec3 vrts; // Vertex store for building `tris`
    Color colr; // Base color
    
    /// Memory Methods ///

    void init_mesh_memory( uint Ntri_ ){
        // Create the `Mesh` struct and allocate memory there

        Ntri = Ntri_;
        Nvtx = Ntri*3;

        // 1. Init mesh
        upldMesh = false;
        upldModl = false;
        sharVrtx = false;
        mesh     = Mesh{};

        mesh.triangleCount = Ntri;
        mesh.vertexCount   = Nvtx;
        
        cout << "GO!: init_mesh_memory( " << mesh.triangleCount << " )" << endl;

        // 2. Init memory
        mesh.vertices = (float* ) MemAlloc(Nvtx*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices  = (ushort*) MemAlloc(Nvtx   * sizeof( ushort ));
        mesh.normals  = (float* ) MemAlloc(Nvtx*3 * sizeof( float  )); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.colors   = (u_char*) MemAlloc(Nvtx*4 * sizeof( u_char )); // 3 vertices, 4 coordinates each (r, g, b, a)
    }

    void load_mesh_buffers( bool loadGeo = true, bool loadColor = false ){
        // Load geometry (and color) info into `mesh` buffers        
        // 2023-08-14: For now assume that the facet indices do NOT change!

        // 0. Init
        ulong k = 0; // Vertex _ counter
        ulong l = 0; // Index __ counter
        ulong m = 0; // Color __ counter

        // 1. Load CPU-side
        for( uint i = 0; i < Ntri; ++i ){
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
        if( !upldMesh ){
            // 3. Initial load to GPU

            cout << "Initial `Mesh` upload, ID: " << mesh.vaoId << endl;

            // UploadMesh( mesh, true );
            UploadMesh( &mesh, true );

            upldMesh = true;
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
        cout << "About to create model ... " << flush;
        modl = LoadModelFromMesh( mesh );
        modl.materials[0] = LoadMaterialDefault();
        modl.materials[0].shader = shdr;
        upldModl = true;
        cout << "Model created!" << endl;
    }

    ~DynaMesh(){
        cout << "." << flush;
    }

    void init_pose(){
        // Set all poses to origin pose
        xfrm = MatrixIdentity();
        Trel = MatrixIdentity();
        Tcur = MatrixIdentity();
    }

    /// Constructors ///

    DynaMesh(){
        // Default Constructor, assumes no default geometry
        init_pose();
    }

    DynaMesh( uint Ntri ){
        // Allocate memory and set default pose
        init_mesh_memory( Ntri );
        init_pose();
    }

    /// Geometry Methods ///

    void push_triangle_w_norms( triPnts tri ){
        // Add one triangle
        Vector3 norm = normal_of_tiangle( tri );  
        tris.push_back( tri );
        nrms.push_back({ norm, norm, norm });
    }
    
    void wipe_geo( bool wipeTris = true, bool wipeColors = false ){
        // Remove all built geometry
        if( wipeTris ){
            tris.clear();
            nrms.clear();
        }
        if( wipeColors ){
            clrs.clear();
        }
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

    void transform_from_parent( const Matrix& parentPose ){
        // Set the absolute pose from the chain of relative poses under the parent
        xfrm = MatrixMultiply( MatrixMultiply( Tcur, Trel ), parentPose );
    }

    /// Appearance & Color ///

    void set_shader( Shader shader ){ shdr = shader; } // Set the shader that will be used for rendering, if applicable

    void set_uniform_color( Color color ){
        // Set base `color` and set all vertex colors to that `color`
        // NOTE: This function assumes that `tris` has been populated!
        colr = color;
        clrs.clear();
        for( size_t i = 0; i < Ntri; ++i ){  clrs.push_back( {colr, colr, colr} );  }
    }

    ///// Drawing Context Methods ////////////////
    // NOTE: Method beyond this point require a drawing context being instantiated before calling!

    /// Rendering ///

    virtual void draw(){
        // Render the mesh
        if( !upldModl )  remodel();
        modl.transform = xfrm;
        // DrawModel( modl, get_posn(), 1.0f, WHITE );
        DrawModel( modl, Vector3Zero(), 1.0f, WHITE );
    }

};
typedef shared_ptr<DynaMesh> dynaPtr;

class Cube : public DynaMesh{ public:
    // Simple Cube

    /// Constructors ///

    Cube( float sideLen, Color color ) : DynaMesh( 12 ){
        // Build and save geo, centered at {0,0,0}

        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm;
        vvec3   V;
        float   halfLen = sideLen/2.0;

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

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};

class Cuboid : public DynaMesh{ public:
    // Simple Rectangular Prism

    /// Constructors ///

    Cuboid( float xSide, float ySide, float zSide, Color color ) : DynaMesh( 12 ){
        // Build and save geo, centered at {0,0,0}

        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm;
        vvec3   V;
        float   halfX = xSide/2.0;
        float   halfY = ySide/2.0;
        float   halfZ = zSide/2.0;

        // 1. Establish vertices
        V.push_back( Vector3{ -halfX, -halfY, -halfZ } );
        V.push_back( Vector3{ -halfX, -halfY,  halfZ } );
        V.push_back( Vector3{ -halfX,  halfY, -halfZ } );
        V.push_back( Vector3{ -halfX,  halfY,  halfZ } );
        V.push_back( Vector3{  halfX, -halfY, -halfZ } );
        V.push_back( Vector3{  halfX, -halfY,  halfZ } );
        V.push_back( Vector3{  halfX,  halfY, -halfZ } );
        V.push_back( Vector3{  halfX,  halfY,  halfZ } );

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

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};

class Cylinder : public DynaMesh{ public:
    // Simple Cylinder

    /// Constructors ///

    Cylinder( float radius, float length, Color color, uint segments = 16 ) : DynaMesh( segments*4 ){
        // Build and save geo
        
        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm, p1, p2, p3, p4, n1, n2;
        vvec3   V;
        float   stepTurn = 2.0f*M_PI/(1.0f*segments);
        float   theta;
        float   halfLen = length/2.0;
        Vector3 c1 = Vector3{ 0.0f, 0.0f,  halfLen };
        Vector3 c2 = Vector3{ 0.0f, 0.0f, -halfLen };
        uint    ip1;


        // 1. Establish vertices
        for( uint i = 1; i <= segments; ++i ){
            theta = 1.0f * i * stepTurn;
            V.push_back( Vector3{ radius*cos( theta ), radius*sin( theta ),  halfLen } );
            V.push_back( Vector3{ radius*cos( theta ), radius*sin( theta ), -halfLen } );
        }

        // 2. Build tris
        for( uint i = 0; i < segments; ++i ){
            ip1 = (i+1)%segments;
            p1 = V[i*2  ];  p3 = V[ip1*2  ];
            p2 = V[i*2+1];  p4 = V[ip1*2+1];
            n1 = Vector3Normalize( Vector3Subtract( p1, c1 ) );
            n2 = Vector3Normalize( Vector3Subtract( p3, c1 ) );

            tris.push_back( {p3, p1, p2} );
            nrms.push_back( {n2, n1, n1} );
            
            tris.push_back( {p3, p2, p4} );
            nrms.push_back( {n2, n1, n2} );

            n1 = Vector3{ 0.0f, 0.0f,  1.0f };
            tris.push_back( {p1, p3, c1} );
            nrms.push_back( {n1, n1, n1} );

            n2 = Vector3{ 0.0f, 0.0f, -1.0f };
            tris.push_back( {p4, p2, c2} );
            nrms.push_back( {n2, n2, n2} );
        }

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};


class Icosahedron : public DynaMesh { public:
    // Good ol' Platonic Solid

    /// Members ///

    // ~ Constants ~
	const float sqrt5 = sqrt( 5.0f ); // ----------------------------------- Square root of 5
	const float phi   = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------- The Golden Ratio
	const float ratio = sqrt( 10.0f + ( 2.0f * sqrt5 ) ) / ( 4.0f * phi ); // ratio of edge length to radius
	
	// ~ Variables ~
	float radius;
	float a; 
	float b; 
    vvec3 V;

    // ~ Appearance ~
    Color baseClr;
    bool  anim;
    float rolVel;
    float ptcVel;
    float yawVel;

    Icosahedron( float rad , const Vector3& cntr, Color color = BLUE, bool active = true ) : DynaMesh( 20 ){
        // Compute the vertices and faces
        // NOTE: This is a building block for the subdivided sphere

        // ~ Geometry Pre-Computation ~
        set_posn( cntr );
        radius = rad;
        // colr   = color;
        a /**/ = ( radius / ratio ) * 0.5;
        b /**/ = ( radius / ratio ) / ( 2.0f * phi );

        // ~ Animation ~
        anim = active;
        float loRate = -0.01f;
        float hiRate =  0.01f;
        rolVel = randf( loRate, hiRate );
        ptcVel = randf( loRate, hiRate );
        yawVel = randf( loRate, hiRate );

        // Define the icosahedron's 12 vertices:
        V.push_back( Vector3{  0,  b, -a } );
        V.push_back( Vector3{  b,  a,  0 } );
        V.push_back( Vector3{ -b,  a,  0 } );
        V.push_back( Vector3{  0,  b,  a } );
        V.push_back( Vector3{  0, -b,  a } );
        V.push_back( Vector3{ -a,  0,  b } );
        V.push_back( Vector3{  0, -b, -a } );
        V.push_back( Vector3{  a,  0, -b } );
        V.push_back( Vector3{  a,  0,  b } );
        V.push_back( Vector3{ -a,  0, -b } );
        V.push_back( Vector3{  b, -a,  0 } );
        V.push_back( Vector3{ -b, -a,  0 } );

        // Define the icosahedron's 20 triangular faces: CCW-out
        push_triangle_w_norms( {V[ 2], V[ 1], V[ 0]} );
        push_triangle_w_norms( {V[ 1], V[ 2], V[ 3]} );
        push_triangle_w_norms( {V[ 5], V[ 4], V[ 3]} );
        push_triangle_w_norms( {V[ 4], V[ 8], V[ 3]} );
        push_triangle_w_norms( {V[ 7], V[ 6], V[ 0]} );
        push_triangle_w_norms( {V[ 6], V[ 9], V[ 0]} );
        push_triangle_w_norms( {V[11], V[10], V[ 4]} );
        push_triangle_w_norms( {V[10], V[11], V[ 6]} );
        push_triangle_w_norms( {V[ 9], V[ 5], V[ 2]} );
        push_triangle_w_norms( {V[ 5], V[ 9], V[11]} );
        push_triangle_w_norms( {V[ 8], V[ 7], V[ 1]} );
        push_triangle_w_norms( {V[ 7], V[ 8], V[10]} );
        push_triangle_w_norms( {V[ 2], V[ 5], V[ 3]} );
        push_triangle_w_norms( {V[ 8], V[ 1], V[ 3]} );
        push_triangle_w_norms( {V[ 9], V[ 2], V[ 0]} );
        push_triangle_w_norms( {V[ 1], V[ 7], V[ 0]} );
        push_triangle_w_norms( {V[11], V[ 9], V[ 6]} );
        push_triangle_w_norms( {V[ 7], V[10], V[ 6]} );
        push_triangle_w_norms( {V[ 5], V[11], V[ 4]} );
        push_triangle_w_norms( {V[10], V[ 8], V[ 4]} );

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }

    void update(){  rotate_RPY( rolVel, ptcVel, yawVel );  } // Rotate
};


class Sphere : public DynaMesh { public:
    // A sphere constructed from a subdivided icosahedron in order to create facets of near-equal area (Good for sims?) 

    Sphere( float rad , const Vector3& cntr, ubyte div = 3, Color color = BLUE ) : 
            DynaMesh( 20 * (div*(div+1)/2 + (div-1)*(div)/2) ) {
        // Compute the vertices and faces
        Icosahedron icos{ rad, cntr };
        Vector3 v0, v1, v2, xTri, yTri, temp, vA, vB, vC, nA, nB, nC;
        for( triPnts& tri : icos.tris ){
            v0 = tri[0];  v1 = tri[1];  v2 = tri[2];
            xTri = Vector3Scale( Vector3Subtract( v1, v0 ), 1.0f/div );
            yTri = Vector3Scale( Vector3Subtract( v2, v0 ), 1.0f/div );

            for( ubyte row = 1; row <= div; ++row ){
                for( ubyte j = row ; j > 0 ; j-- ){ // Construct the v0-pointing tris
                    vA = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j  ), (float) (row-j  ), xTri, yTri ) );
                    vB = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j-1), (float) (row-j+1), xTri, yTri ) );
                    vC = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j-1), (float) (row-j  ), xTri, yTri ) );
                    nA = Vector3Normalize( vA );
                    nB = Vector3Normalize( vB );
                    nC = Vector3Normalize( vC );
                    vA = Vector3Scale( nA, rad );
                    vB = Vector3Scale( nB, rad );
                    vC = Vector3Scale( nC, rad );
                    tris.push_back( {vA, vB, vC} );
                    nrms.push_back( {nA, nB, nC} );
                }
                for( ubyte j = row - 1 ; j > 0 ; j-- ){ // Construct the anti-v0-pointing tris
                    vA = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j  ), (float) (row-1-j  ), xTri, yTri ) );
                    vB = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j  ), (float) (row-1-j+1), xTri, yTri ) );
                    vC = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j-1), (float) (row-1-j+1), xTri, yTri ) );
                    nA = Vector3Normalize( vA );
                    nB = Vector3Normalize( vB );
                    nC = Vector3Normalize( vC );
                    vA = Vector3Scale( nA, rad );
                    vB = Vector3Scale( nB, rad );
                    vC = Vector3Scale( nC, rad );
                    tris.push_back( {vA, vB, vC} );
                    nrms.push_back( {nA, nB, nC} );
                }
            }
        }

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};


class EllipticalTorusXY : public DynaMesh { public:
    // Generate an elliptical torus in the XY plane, Can be rotated
    EllipticalTorusXY( float a, float b, float dia, uint rotationRes, uint revolveRes, Color color = BLUE ) : DynaMesh( rotationRes * revolveRes * 2 ) {
        // Compute the vertices and faces
        Vector3 circCntr_i, circCntr_ip1, axis_i, axis_ip1;
        Vector3 rad_i, rad_ip1;
        Vector3 p1, p2, p3, p4, n1, n2, n3, n4;

        float theta   = 0.0f, phi;
        float rotStep = (2.0f * M_PI) / (1.0f * rotationRes);
        float revStep = (2.0f * M_PI) / (1.0f * revolveRes );

        for( uint i = 0; i < rotationRes; ++i ){

            circCntr_i   = {a*cosf(theta)        , b*sinf(theta)        , 0.0f};
            circCntr_ip1 = {a*cosf(theta+rotStep), b*sinf(theta+rotStep), 0.0f};
            axis_i /*-*/ = Vector3CrossProduct( Vector3{0.0f,0.0f,-1.0f}, circCntr_i   );
            axis_ip1     = Vector3CrossProduct( Vector3{0.0f,0.0f,-1.0f}, circCntr_ip1 );
            phi /*----*/ = 0.0f;
            rad_i /*--*/ = Vector3Scale( Vector3Normalize( circCntr_i   ), dia/2.0f );
            rad_ip1 /**/ = Vector3Scale( Vector3Normalize( circCntr_ip1 ), dia/2.0f );

            for( uint j = 0; j < revolveRes; ++j ){

                p1 = Vector3Add( circCntr_i  , Vector3RotateByAxisAngle( rad_i  , axis_i  , phi         ) );
                p2 = Vector3Add( circCntr_ip1, Vector3RotateByAxisAngle( rad_ip1, axis_ip1, phi         ) );
                p3 = Vector3Add( circCntr_i  , Vector3RotateByAxisAngle( rad_i  , axis_i  , phi+revStep ) );
                p4 = Vector3Add( circCntr_ip1, Vector3RotateByAxisAngle( rad_ip1, axis_ip1, phi+revStep ) );
                n1 = Vector3Normalize( Vector3Subtract( p1, circCntr_i   ) );
                n2 = Vector3Normalize( Vector3Subtract( p2, circCntr_ip1 ) );
                n3 = Vector3Normalize( Vector3Subtract( p3, circCntr_i   ) );
                n4 = Vector3Normalize( Vector3Subtract( p4, circCntr_ip1 ) );

                tris.push_back( {p3, p1, p2} );
                nrms.push_back( {n3, n1, n2} );
                
                tris.push_back( {p3, p2, p4} );
                nrms.push_back( {n3, n2, n4} );

                phi += revStep;
            }
            theta += rotStep;
        }
        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};

////////// COMPLEX MESH TOYS ///////////////////////////////////////////////////////////////////////

class CompositeModel{ public:
    // Contains multiple `DynaMesh` parts

    /// Members ///

    Matrix /*----*/ xfrm; // --- Pose of the entire model
    vector<dynaPtr> parts; // -- Drawable components
    Color /*-----*/ prtColor; // Main color of meshes

    /// Constructor(s) ///

    CompositeModel(){
        // Default pose is the origin
        xfrm     = MatrixIdentity();
        prtColor = BLUE;
    }

    /// Methods ///

    void set_shader( Shader shader ){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->set_shader( shader );  }
    }

    void set_position( const Vector3& posn ){
        // Set the position of the model
        xfrm = set_posn( xfrm, posn );
    }

    void set_pose( const Matrix& pose ){
        // Set the pose of the model
        xfrm = pose;
    }

    Vector3 get_position(){
        // Get the position of the model
        return get_posn( xfrm );
    }

    void set_part_poses(){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->transform_from_parent( xfrm );  }
    }

    Matrix set_part_pose( ulong i ){
        // Set the shader for all parts
        if( i < parts.size() ){  
            parts[i]->transform_from_parent( xfrm );  
            return parts[i]->xfrm;
        }
        return MatrixError();
    }

    void draw(){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->draw();  }
    }

    size_t add_component( dynaPtr part ){
        // Add a component and return the current part count
        parts.push_back( part );
        return parts.size();
    }
};



////////// LIGHTING ////////////////////////////////////////////////////////////////////////////////

struct Lighting{
    // Container struct for light(s) and associated shader

    /// Members ///
    Shader shader;
    float  ambientColor[4];
    int    ambientLoc;
    Light  light;

    /// Constructor ///
    Lighting(){
        // Load basic lighting shader
        shader = LoadShader( TextFormat("shaders/lighting.vs") ,
                             TextFormat("shaders/lighting.fs") );
        // Get some required shader locations
        shader.locs[ SHADER_LOC_VECTOR_VIEW ] = GetShaderLocation( shader, "viewPos" );
        // Ambient light level (some basic lighting)
        ambientColor[0] = 0.25f;
        ambientColor[1] = 0.25f;
        ambientColor[2] = 0.25f;
        ambientColor[3] = 0.50f;
        ambientLoc /**/ = GetShaderLocation( shader, "ambient" );
        SetShaderValue( shader, ambientLoc, ambientColor, SHADER_UNIFORM_VEC4 );
        // Using just 1 point lights
        light = CreateLight(
            LIGHT_POINT, 
            Vector3{ 20.0, 20.0, 20.0 }, 
            Vector3Zero(), 
            Color{ 255, 255, 255, 125 }, 
            shader
        );
    }

    /// Methods ///

    void update(){  UpdateLightValues( shader, light );  } // Update the light

    void set_camera_posn( Camera& cam ){
        // Tell the shader where the camera is
        SetShaderValue( shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &cam.position.x, SHADER_UNIFORM_VEC3 );
    }
};



////////// PRINTING FUNCTIONS //////////////////////////////////////////////////////////////////////

ostream& operator<<( ostream& os , const Vector3& vec ) { 
    // ostream '<<' operator for Raylib Color
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "{X: "  << ((float) vec.x);
    os << ", Y: " << ((float) vec.y);
    os << ", Z: " << ((float) vec.z);
    os << "}";
    return os; // You must return a reference to the stream!
}

ostream& operator<<( ostream& os , const Vector2& vec ) { 
    // ostream '<<' operator for Raylib Color
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "{X: "  << ((float) vec.x);
    os << ", Y: " << ((float) vec.y);
    os << "}";
    return os; // You must return a reference to the stream!
}

#endif /* RL_TOYBOX_H */ 
#ifndef RL_TOYBOX_H
#define RL_TOYBOX_H

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Includes ///////////////////////////////////

/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <vector>     
using std::vector; 
#include <array>
using std::array;
#include <algorithm>
using std::min;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>

///// Type Aliases ///////////////////////////////
typedef unsigned char /*---*/ ubyte;
typedef unsigned long /*---*/ ulong;
typedef vector<float> /*---*/ vf;
typedef vector<vector<float>> vvf;



////////// SHADERS /////////////////////////////////////////////////////////////////////////////////

void setModelShader( Model* m, Shader* s ){
    // Assign a shader to a material index within the model
    // Author: Bedroom Coder, https://bedroomcoders.co.uk/cell-shader-revisited/
    for( int i = 0; i < m->materialCount; i++ ){
        m->materials[i].shader = *s;
    }
}


////////// VECTOR MATH /////////////////////////////////////////////////////////////////////////////

float vec3_mag( Vector3 vec ){  return sqrt( pow( vec.x, 2 ) + pow( vec.y, 2 ) + pow( vec.z, 2 ) );  } // Magnitude of `vec`

Vector3 vec3_divide( Vector3 vec, float div ){
    // Divide each element by a scalar
	return Vector3{
		vec.x / div,
		vec.y / div,
		vec.z / div
	};
}

Vector3 vec3_unit( Vector3 vec ){
    // Get the unit vector in the direction of `vec`
	float mag = vec3_mag( vec );
	if( mag == 0.0 )  
		return vec;
	else 
		return vec3_divide( vec, mag );
}

Vector3 vec3_mult( Vector3 vec, float factor ){
    // Multiply each element by a scalar
	return Vector3{
		vec.x * factor,
		vec.y * factor,
		vec.z * factor
	};
}



////////// IMAGE & COLOR ///////////////////////////////////////////////////////////////////////////

Color get_image_pixel_color_at( Color* img, ulong row, ulong col, ulong Ncols ){
    // Get the Color object representing the color values at `row` and `col`
    // https://www.reddit.com/r/raylib/comments/pol80c/comment/hcxhhhf
    // WARNING: This function does not check image bounds!
    Color  rtnClr = img[ row * Ncols + col ];
    return rtnClr;
}

float get_image_pixel_red_intensity( Color* img, ulong row, ulong col, ulong Ncols ){
    // Get the red intensity at the given `row` and `col`
    return 1.0f * get_image_pixel_color_at( img, row, col, Ncols ).r / 255.0f;
}

vvf get_subimage_red_intensity( Color* img, ulong Ncols, ulong rowUL, ulong colUL, ulong rowLR, ulong colLR ){
    vvf rtnImg;
    vf  rowRtn;
    for( ulong i = rowUL; i < rowLR; i++ ){
        rowRtn.clear();
        for( ulong j = colUL; j < colLR; j++ ){
            rowRtn.push_back( get_image_pixel_red_intensity( img, i, j, Ncols ) );
        }   
        rtnImg.push_back( rowRtn );
    }
    return rtnImg;
}

ostream& operator<<( ostream& os , const Color& clr ) { 
    // ostream '<<' operator for Raylib Color
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "{R: "  << ((int) clr.r);
    os << ", G: " << ((int) clr.g);
    os << ", B: " << ((int) clr.b);
    os << ", A: " << ((int) clr.a);
    os << "}";
    return os; // You must return a reference to the stream!
}





////////// TRIMESH /////////////////////////////////////////////////////////////////////////////////

///// Unshared Vertex Helpers ////////////////////

void init_mesh( Mesh& mesh, ulong Ntri ){
    // Allocate memory in the mesh for triangles with unshared points
    ulong Nvrt = Ntri * 3;
    mesh = Mesh{};
    // Init geo memory
    mesh.triangleCount = Ntri;
    mesh.vertexCount   = Nvrt;
    cout << "Allocating " << Nvrt*3*sizeof(float) << " bytes for vertices ..." << endl;
    mesh.vertices /**/ = (float *)MemAlloc(Nvrt*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
    cout << "Allocating " << Nvrt*sizeof(ushort) << " bytes for indices ..." << endl;
    mesh.indices /*-*/ = (ushort *)MemAlloc(Nvrt*sizeof(ushort));
}

void init_mesh_normals( Mesh& mesh, ulong Ntri ){
    // Allocate memory in the mesh for normals with unshared points
    ulong Nvrt = Ntri * 3;
    mesh.normals = (float *)MemAlloc(Nvrt*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
}

///// Shared Vertex Helpers ////////////////////

void init_mesh( Mesh& mesh, ulong Npts, ulong Ntri ){
    // Allocate memory in the mesh for triangles with shared points
    ulong Nvrt = Ntri * 3;
    mesh = Mesh{};
    // Init geo memory
    mesh.triangleCount = Ntri;
    mesh.vertexCount   = Nvrt;
    cout << "Allocating " << Npts*3*sizeof(float) << " bytes for vertices ..." << endl;
    mesh.vertices /**/ = (float *)MemAlloc(Npts*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
    cout << "Allocating " << Nvrt*sizeof(ushort) << " bytes for indices ..." << endl;
    mesh.indices /*-*/ = (ushort *)MemAlloc(Nvrt*sizeof(ushort));
}


class TriModel{ public:
    // Container class for simple models

    // Triangles //
    vector<array<Vector3,3>> tris; //- Triangle data
    vector<Vector3> /*----*/ pnts; //- Point data
    vector<array<ushort,3>>  ndcs; //- Index data

    // Model //
	Mesh  mesh; //- Raylib mesh geometry
	Model model; // Raylib drawable model

    // Pose //
    float  x; //- World X pos
	float  y; //- World Y pos
	float  z; //- World Z pos
	// Matrix R; //  World rotation
	float  r; //- Local roll  angle
	float  p; //- Local pitch angle
	float  w; //- Local yaw   angle
	Matrix mx; // Local pitch
	Matrix my; // Local yaw
	Matrix mz; // Local roll
	Matrix T; //- Orientation
	Matrix dT; //- Change in orientation

    ///// Unshared Vertex Ops ////////////////////

    void load_tri( const Vector3& v1, const Vector3& v2, const Vector3& v3 ){
        // Load one triangle, Right hand rule
        array<Vector3,3> pushArr;
        pushArr[0] = v1;
        pushArr[1] = v2;
        pushArr[2] = v3;
        tris.push_back( pushArr );
    }

    void build_mesh_unshared(){
        // Load the triangle data into the mesh
        cout << "There are " << tris.size() << " triangles!";
        ulong  k = 0;
        ushort l = 0;
        for( ulong i = 0; i < tris.size(); i++ ){
            for( ubyte j = 0; j < 3; j++ ){
                mesh.vertices[k] = tris[i][j].x;  k++;
                mesh.vertices[k] = tris[i][j].y;  k++;
                mesh.vertices[k] = tris[i][j].z;  k++;
                mesh.indices[l]  = l; /*------*/  l++; // WARNING: UNOPTIMIZED FOR SHARED VERTICES
            }
        }
    }

    void build_normals_flat_unshared(){
        // Create normals for flat shading (unshared vertices)
        ulong   k = 0;
        Vector3 v1, v2, v3, e1, e2, nrm;
        // Allocate memory for normals
        init_mesh_normals( mesh, tris.size() );
        // Compute a vector perpendiculat to each triangle and store its components
        for( ulong i = 0; i < tris.size(); i++ ){
            v1  = tris[i][0];
            v2  = tris[i][1];
            v3  = tris[i][2];
            e1  = Vector3Subtract( v1, v2 );
            e2  = Vector3Subtract( v3, v2 );
            nrm = Vector3CrossProduct( e1, e2 );
            for( ubyte j = 0; j < 3; j++ ){
                mesh.normals[k] = nrm.x;  k++;
                mesh.normals[k] = nrm.y;  k++;
                mesh.normals[k] = nrm.z;  k++;
            }
        }
    }

    ///// Shared Vertex Ops ////////////////////

    void load_pnt( const Vector3& vec ){
        // Load one point
        pnts.push_back( vec );
    }

    void load_indices( ushort i1, ushort i2, ushort i3 ){
        // Load index triple, Right hand rule
        array<ushort,3> pushArr;
        pushArr[0] = i1;
        pushArr[1] = i2;
        pushArr[2] = i3;
        ndcs.push_back( pushArr );
    }

    void build_mesh_shared(){
        // Load the triangle data into the mesh
        ulong  k = 0;
        ushort l = 0;
        for( ulong i = 0; i < pnts.size(); i++ ){
            mesh.vertices[k] = pnts[i].x;  k++;
            mesh.vertices[k] = pnts[i].y;  k++;
            mesh.vertices[k] = pnts[i].z;  k++;
        }
        for( ulong i = 0; i < ndcs.size(); i++ ){
            for( ubyte j = 0; j < 3; j++ ){
                mesh.indices[l] = ndcs[i][j];  l++;
            }
        }
    }

    ///// Constructors ///////////////////////////

    TriModel(){
        // Default constructor
        mesh = Mesh{};
    }

    void init_pose(){
        // Init pose
		x = 0.0; // World X pos
		y = 0.0; // World Y pos
		z = 0.0; // World Z pos
		r = 0.0; // Local roll  angle
		p = 0.0; // Local pitch angle
		w = 0.0; // Local yaw   angle
        dT = MatrixIdentity();
        T  = MatrixIdentity();
    }

    TriModel( ulong Ntri ){
        // Unshared Constructor
        init_mesh( mesh, Ntri );
        init_pose();
    }

    TriModel( ulong Npts, ulong Ntri ){
        // Shared Constructor
        init_mesh( mesh, Npts, Ntri );
        init_pose();
    }

    ~TriModel(){
        // Free all allocated memory
        if( mesh.vertices )  free( mesh.vertices );
        if( mesh.indices  )  free( mesh.indices  );
        if( mesh.normals  )  free( mesh.normals  );
    }

    ///// Pose Math //////////////////////////////

    Vector3 get_XYZ(){
		// Set the world XYZ position of the model
        return Vector3{x,y,z};
	}

    void set_XYZ( float x_, float y_, float z_ ){
		// Set the world XYZ position of the model
		x = x_;
		y = y_;
		z = z_;
	}

    void rotate_RPY( float r_, float p_, float y_ ){
		// Increment the world Roll, Pitch, Yaw of the model
		r += r_;
		p += p_;
		w += y_;
        mz = MatrixRotateZ( r_ ); // Roll  about local Z
		mx = MatrixRotateX( p_ ); // Pitch about local X
        my = MatrixRotateY( y_ ); // Yaw   about local Y
        dT = MatrixMultiply( MatrixMultiply( my, mx ), mz );
        T  = MatrixMultiply( dT, T );
        model.transform = T;
	}

    void z_thrust( float d = 0.0 ){
		// Advance a plane model in the forward direction (local Z)
		Vector3 vec = Vector3Transform( Vector3{ 0.0, 0.0, d }, T );
		x += vec.x;
		y += vec.y;
		z += vec.z;
	}


    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_mesh(){
		// Send triangle mesh geometry to RayLib, needed for drawing
        cout << "\t\t\t`UploadMesh()` ..." << endl;
		UploadMesh( &mesh, true );
		// UploadMesh( &mesh, false );
        cout << "\t\t\t`LoadModelFromMesh()` ..." << endl;
    	model = LoadModelFromMesh( mesh );
        model.transform = T;
	}

    void load_geo(){
        // Get the model ready for drawing
        if( tris.size() > 0 ){
            build_mesh_unshared();
            load_mesh();
        }else if( pnts.size() > 0 ){
            build_mesh_shared();
            load_mesh();
        }else{  cout << "Mesh not initialized shared or unshared!" << endl;  }
    }

    void draw(){
        cout << "Please implement `draw` for derived class!" << endl;
    };
};



////////// DELTA GLIDER ////////////////////////////////////////////////////////////////////////////

class DeltaGlider : public TriModel{ public:
    // A fun little space plane

    Color sldClr = BLACK; // SKYBLUE;
    Color linClr = RAYWHITE; // RAYWHITE; // SKYBLUE; // LIGHTGRAY; // LIME; // WHITE; // BLACK;

    DeltaGlider( float wingspan = 10.0f ) : TriModel( 8 ){
        // 

        float fusFrac   = 0.5;
        float sweptFrac = 0.75;
        float thickFrac = 0.25;

        // Using unshared vertices for flat shading

        load_tri( // Left  Front Top
            Vector3{  0.0f        ,  0.0f                 , 0.0f                + wingspan*sweptFrac/2.0f }, // 0, Front
            Vector3{ -wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }, // 4, Left wingtip
            Vector3{  0.0f        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f } //  1, Top peak
        );
        load_tri( // Left  Rear  Top
            Vector3{  0.0f        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }, // 1, Top peak
            Vector3{ -wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }, // 4, Left wingtip
            Vector3{  0.0f        ,  0.0f                 , -wingspan*fusFrac   + wingspan*sweptFrac/2.0f } //  3, Back
        );
        load_tri( // Right Front Top
            Vector3{ +wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }, // 5, Right wingtip
            Vector3{  0.0f        ,  0.0f                 , 0.0f                + wingspan*sweptFrac/2.0f }, // 0, Front
            Vector3{  0.0f        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }  // 1, Top peak
        );
        load_tri( // Right Rear  Top
            Vector3{ +wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }, // 5, Right wingtip
            Vector3{  0.0f        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }, // 1, Top peak
            Vector3{  0.0f        ,  0.0f                 , -wingspan*fusFrac   + wingspan*sweptFrac/2.0f } //  3, Back
        );
        load_tri( // Left  Front Bottom
            Vector3{  0.0f        ,  0.0f                 , 0.0f                + wingspan*sweptFrac/2.0f }, // 0, Front
            Vector3{  0.0f        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }, // 2, Bottom peak
            Vector3{ -wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }  // 4, Left wingtip
        );
        load_tri( // Left  Rear  Bottom
            Vector3{  0.0f        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }, // 2, Bottom peak
            Vector3{  0.0f        ,  0.0f                 , -wingspan*fusFrac   + wingspan*sweptFrac/2.0f }, // 3, Back
            Vector3{ -wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }  // 4, Left wingtip
        );
        load_tri( // Right Front Bottom
            Vector3{  0.0f        ,  0.0f                 , 0.0f                + wingspan*sweptFrac/2.0f }, // 0, Front
            Vector3{ +wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }, // 5, Right wingtip
            Vector3{  0.0f        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }  // 2, Bottom peak
        );
        load_tri( // Right Rear  Bottom
            Vector3{  0.0f        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f }, // 2, Bottom peak
            Vector3{ +wingspan/2  ,  0.0f                 , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f }, // 5, Right wingtip
            Vector3{  0.0f        ,  0.0f                 , -wingspan*fusFrac   + wingspan*sweptFrac/2.0f }  // 3, Back
        );
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        build_mesh_unshared();
        build_normals_flat_unshared();
        load_mesh();
    }

    void draw(){
        // Draw the model
        DrawModel(      model, Vector3{x, y, z}, 1.00, sldClr );  
        DrawModelWires( model, Vector3{x, y, z}, 1.02, linClr );
    }

};



////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

class FlightFollowThirdP_Camera : public Camera3D{ public:
	// Aircraft drags the camera like in games

	Vector3 trgtCenter; // Position of the target
	float   offset_d; // - Desired camera offset in meters
    Matrix  trgtXform; //- Orientation of the target

    static constexpr float dDrawMin = RL_CULL_DISTANCE_NEAR;
    static constexpr float dDrawMax = RL_CULL_DISTANCE_FAR;

    FlightFollowThirdP_Camera( float desiredOffset_m, Vector3 tCenter, Matrix tXform ) : Camera3D(){
        // Set follower params 
        trgtCenter = tCenter; 
		offset_d   = desiredOffset_m;
		trgtXform  = tXform; // -------- Orientation of the target
        // Set inherited params
        position   = Vector3Add( tCenter, Vector3Transform( Vector3{0.0, 0.0, -offset_d}, tXform) );
        target     = trgtCenter;
        up /*---*/ = Vector3{ 0.0, 0.0, 1.0 };
        fovy /*-*/ = 45.0f;
        projection = 0;
    }

    void update_target_position( Vector3 tCenter ){  trgtCenter = tCenter;  } // Point the camera

    void advance_camera(){
		// Move the camera after all the target updates are in
        Vector3 trgtDiff = Vector3Subtract( position, trgtCenter );
        float   sepDist  = min( vec3_mag( trgtDiff ), offset_d );
		Vector3 dragVec  = vec3_mult( 
			vec3_unit( trgtDiff ), 
			sepDist
		);
		position = Vector3Add( trgtCenter, dragVec );
		target   = trgtCenter;
	}

    bool inside_FOV( const Vector3& pnt ){
        // Return true if the ray from the camera to the `pnt` is within the FOV (conservative)
        Vector3 vLook = Vector3Subtract( target, position );
        Vector3 vRayP = Vector3Subtract( pnt   , position );
        if( Vector3Angle( vLook, vRayP ) * RAD2DEG <= fovy )  
            return true;
        else
            return false;
    }

    float signed_distance_to_frustrum( const Vector3& pnt ){
        if( inside_FOV( pnt ) ){
            return Vector3Distance( pnt, position ) - dDrawMax;
        }else{
            return nanf("");
        }
    }
};

#endif /* RL_TOYBOX_H */ 
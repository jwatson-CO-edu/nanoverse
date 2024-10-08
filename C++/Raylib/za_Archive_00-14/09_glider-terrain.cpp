// g++ 09_glider-terrain.cpp -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <stdlib.h>  // srand, rand
#include <time.h>
#include <array>     
using std::array;  
#include <vector>     
using std::vector;  
#include <iostream>
using std::cout, std::endl;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

///// Type Aliases ///////////////////////////////
typedef unsigned long  ulong;
typedef unsigned short ushort;
typedef unsigned char  ubyte;



////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

float randf( float lo, float hi ){
    // Return a pseudo-random number between `lo` and `hi`
    float span = hi - lo;
    return lo + randf() * span;
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed



////////// VECTOR MATH /////////////////////////////////////////////////////////////////////////////

float vec3_mag( Vector3 vec ){  return sqrt( pow( vec.x, 2 ) + pow( vec.y, 2 ) + pow( vec.z, 2 ) );  }

Vector3 vec3_divide( Vector3 vec, float div ){
	return Vector3{
		vec.x / div,
		vec.y / div,
		vec.z / div
	};
}

Vector3 vec3_unit( Vector3 vec ){
	float mag = vec3_mag( vec );
	if( mag == 0.0 )  
		return vec;
	else 
		return vec3_divide( vec, mag );
}

Vector3 vec3_mult( Vector3 vec, float factor ){
	return Vector3{
		vec.x * factor,
		vec.y * factor,
		vec.z * factor
	};
}



////////// TRIMESH /////////////////////////////////////////////////////////////////////////////////

void init_mesh( Mesh& mesh, ulong Ntri ){
    // Allocate memory in the mesh for triangles with unshared points
    ulong Nvrt = Ntri * 3;
    mesh = Mesh{};
    // Init geo memory
    mesh.triangleCount = Ntri;
    mesh.vertexCount   = Nvrt;
    mesh.vertices /**/ = (float *)MemAlloc(Nvrt*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
    mesh.indices /*-*/ = (ushort *)MemAlloc(Nvrt*sizeof(ushort));
}

void init_mesh( Mesh& mesh, ulong Npts, ulong Ntri ){
    // Allocate memory in the mesh for triangles with shared points
    ulong Nvrt = Ntri * 3;
    mesh = Mesh{};
    // Init geo memory
    mesh.triangleCount = Ntri;
    mesh.vertexCount   = Nvrt;
    mesh.vertices /**/ = (float *)MemAlloc(Npts*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
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

    ///// Geometry Data Manip ////////////////////

    void load_tri( const Vector3& v1, const Vector3& v2, const Vector3& v3 ){
        // Load one triangle, Right hand rule
        array<Vector3,3> pushArr;
        pushArr[0] = v1;
        pushArr[1] = v2;
        pushArr[2] = v3;
        tris.push_back( pushArr );
    }

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

    void build_mesh_unshared(){
        // Load the triangle data into the mesh
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



////////// TERRAIN /////////////////////////////////////////////////////////////////////////////////

class TerrainPlate : public TriModel { public:
    // Rectangular plate of randomized terrain

    vector<vector<Vector3>> pts; // -- Grid points in 3D space
    float /*-------------*/ scl; // -- Scale of each cell
    ulong /*-------------*/ M; // ---- Number of rows
    ulong /*-------------*/ N; // ---- Number of cells per row
    Color /*-------------*/ gndClr; // Triangle fill color
    Color /*-------------*/ linClr; // Triangle line color
    float /*-------------*/ offset; // Z bump for lines
    Vector3 /*-----------*/ posn1; //- Facet drawing origin
    Vector3 /*-----------*/ posn2; //- Line  drawing origin

    TerrainPlate( float scale = 10.0f, ulong Mrows = 10, ulong Ncols = 10 ) : TriModel( (Mrows-1)*(Ncols-1)*2 ){
        // Generate points and load triangles
        
        // 0. Init
        M /**/ = Mrows;
        N /**/ = Ncols;
        scl    = scale;
        offset = scale/200.0f;
        gndClr = GREEN;
        linClr = BLACK;
        posn1  = Vector3{ 0.0f, 0.0f, 0.0f   };
        posn2  = Vector3{ 0.0f, 0.0f, offset };

        // 1. Generate points
        vector<Vector3> row;
        for( ulong i = 0; i < M; i++ ){
            row.clear();
            for( ulong j = 0; j < N; j++ ){
                row.push_back(  Vector3{ 
                    j*scl + randf( -scl*0.25, +scl*0.25 ), 
                    i*scl + randf( -scl*0.25, +scl*0.25 ),
                    randf()*scl 
                }  );
            }
            pts.push_back( row );
        }

        // 2. Build triangles
        for( ulong i = 0; i < M-1; i++ ){
            for( ulong j = 0; j < N-1; j++ ){
                Vector3 v1, v2, v3, v4;
                // Load points
                v1 = pts[i  ][j  ];
                v2 = pts[i  ][j+1];
                v3 = pts[i+1][j  ];
                v4 = pts[i+1][j+1];
                // Randomize cross right or cross left
                if( randf() < 0.5f ){
                    load_tri( v1, v2, v3 );
                    load_tri( v3, v2, v4 );
                }else{
                    load_tri( v1, v2, v4 );
                    load_tri( v3, v1, v4 );
                }
            }
        }
    }

    void draw(){
        // Draw facets, shift up, draw lines
        DrawModel(      model, posn1, 1.0, gndClr );  
        DrawModelWires( model, posn2, 1.0, linClr );
    }
};



////////// DELTA GLIDER ////////////////////////////////////////////////////////////////////////////

class DeltaGlider : public TriModel{ public:
    // A fun little space plane

    Color sldClr = SKYBLUE;
    Color linClr = BLACK;

    DeltaGlider( float wingspan = 10.0f ) : TriModel( 6, 8 ){
        // 

        float fusFrac   = 0.5;
        float sweptFrac = 0.75;
        float thickFrac = 0.25;

        load_pnt( Vector3{  0.0f      ,  0.0f                , 0.0f                + wingspan*sweptFrac/2.0f } ); // 0, Front
        load_pnt( Vector3{  0.0f      , +wingspan*thickFrac/2, -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f } ); // 1, Top peak
        load_pnt( Vector3{  0.0f      , -wingspan*thickFrac/2, -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0f } ); // 2, Bottom peak
        load_pnt( Vector3{  0.0f      ,  0.0f                , -wingspan*fusFrac   + wingspan*sweptFrac/2.0f } ); // 3, Back
        load_pnt( Vector3{ -wingspan/2,  0.0f                , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f } ); // 4, Left wingtip
        load_pnt( Vector3{ +wingspan/2,  0.0f                , -wingspan*sweptFrac + wingspan*sweptFrac/2.0f } ); // 5, Right wingtip
        
        load_indices( 0,4,1 ); // Left  Front Top
        load_indices( 1,4,3 ); // Left  Rear  Top
        load_indices( 5,0,1 ); // Right Front Top
        load_indices( 5,1,3 ); // Right Rear  Top
        load_indices( 0,2,4 ); // Left  Front Bottom
        load_indices( 2,3,4 ); // Left  Rear  Bottom
        load_indices( 0,5,2 ); // Right Front Bottom
        load_indices( 2,5,3 ); // Right Rear  Bottom
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

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

    void update_target_position( Vector3 tCenter ){  trgtCenter = tCenter;  }

    void advance_camera(){
		// Move the camera after all the target updates are in
		Vector3 dragVec = vec3_mult( 
			vec3_unit( Vector3Subtract( position, trgtCenter ) ), 
			offset_d 
		);
		position = Vector3Add( trgtCenter, dragVec );
		target   = trgtCenter;
	}
};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    /// Scene Init: Pre-Window ///
    TerrainPlate terrain{ 10.0f, 25, 25 };
    DeltaGlider  glider{ 10.0f };
	float /*--*/ frameRotateRad = 3.1416/120.0;
    float /*--*/ frameThrust    = 3.0/60.0;

    /// Window Init ///
    InitWindow( 800, 450, "Terrain Gen + Glider" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Scene Init: Post-Window ///
    terrain.load_geo(); 
    glider.load_geo();
    glider.set_XYZ( 25*10/2.0f, 25*10/2.0f, 10.0f );
    glider.rotate_RPY( 0.0, 3.1416/2.0, 0.0 );

    FlightFollowThirdP_Camera camera{
        25.0,
		glider.get_XYZ(),
		glider.T
    };


    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        // Keyboard input
		if( IsKeyDown( KEY_Z     ) ){  glider.rotate_RPY(  0.0           ,  0.0,             frameRotateRad );  }
		if( IsKeyDown( KEY_X     ) ){  glider.rotate_RPY(  0.0           ,  0.0,            -frameRotateRad );  }
        if( IsKeyDown( KEY_LEFT  ) ){  glider.rotate_RPY( -frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_RIGHT ) ){  glider.rotate_RPY(  frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_UP    ) ){  glider.rotate_RPY(  0.0           ,  frameRotateRad,  0.0            );  }
		if( IsKeyDown( KEY_DOWN  ) ){  glider.rotate_RPY(  0.0           , -frameRotateRad,  0.0            );  }

		// gamepad input
		if( IsGamepadAvailable(0) ){
			glider.rotate_RPY( 0.0, 0.0, -frameRotateRad*GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X) );
			glider.rotate_RPY( 0.0, -frameRotateRad*GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y), 0.0 );
		}

        camera.update_target_position( glider.get_XYZ() );
		camera.advance_camera();

        ///// DRAW LOOP //////////////////////////
        terrain.draw();

        glider.z_thrust( frameThrust );
        glider.draw();

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}
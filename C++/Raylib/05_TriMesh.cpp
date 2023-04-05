// gcc 05_TriMesh.cpp -lraylib

/* ////////// DEV PLAN //////////
[ ] Investigate: https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/
*/

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <array>     
using std::array;  
#include <vector>     
using std::vector;  
#include <iostream>
using std::cout, std::endl;

// #include "raylib.h"
// #include "raymath.h"
// #include "rlgl.h" // `rlDisableBackfaceCulling`
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

typedef unsigned long  ulong;
typedef unsigned short ushort;
typedef unsigned char  ubyte;


////////// TRIMESH /////////////////////////////////////////////////////////////////////////////////

class TriModel{
public:
    // Container class for simple models
    // WARNING: UNOPTIMIZED FOR SHARED VERTICES
    // FIXME: ALLOW FOR SHARED VERTICES

    // Triangles //
    ulong /*--------------*/ N_tri; //- Number of triangles
    ulong /*--------------*/ N_vrt; // Number of vertices
    vector<array<Vector3,3>> tris; //-- Triangle data
    // ulong /*--------------*/ tDex; //- Index offset for triangle coords

    // Model //
	// Mesh  mesh{}; //- Raylib mesh geometry
	Mesh  mesh; //- Raylib mesh geometry
	Model model; // Raylib drawable model

    // Pose //
    float  x; //- World X pos
	float  y; //- World Y pos
	float  z; //- World Z pos
	Matrix R; //  World rotation
	float  r; //- Local roll  angle
	float  p; //- Local pitch angle
	float  w; //- Local yaw   angle
	Matrix mx; // Local pitch
	Matrix my; // Local yaw
	Matrix mz; // Local roll
	Matrix T; //- World orientation


    ///// Geometry Data Manip ////////////////////

    void load_tri( const Vector3& v1, const Vector3& v2, const Vector3& v3 ){
        // Load one triangle, Right hand rule
        array<Vector3,3> pushArr;
        pushArr[0] = v1;
        pushArr[1] = v2;
        pushArr[2] = v3;
        tris.push_back( pushArr );
        // tDex++;
    }

    void per_tri_normals(){
        // Just make all the normals normal to their own triangle
        ulong   k = 0;
        Vector3 n;
        for( ulong i = 0; i < N_tri; i++ ){
            n = Vector3CrossProduct( Vector3Subtract(tris[i][0],tris[i][1]), Vector3Subtract(tris[i][2],tris[i][1]));
            cout << "\t\t\t\tNormal @ " << k << " {" << 
                n.x << ", " <<
                n.y << ", " <<
                n.z << 
            "}" << endl;
            for( ubyte j = 0; j < 3; j++ ){
                mesh.normals[k] = n.x;  k++;
                mesh.normals[k] = n.y;  k++;
                mesh.normals[k] = n.z;  k++;
            }
        }
    }

    void build_mesh(){
        // Load the triangle data into the mesh
        ulong  k = 0;
        ushort l = 0;
        for( ulong i = 0; i < N_tri; i++ ){
            for( ubyte j = 0; j < 3; j++ ){
                mesh.vertices[k] = tris[i][j].x;  k++;
                mesh.vertices[k] = tris[i][j].y;  k++;
                mesh.vertices[k] = tris[i][j].z;  k++;
                mesh.indices[l]  = l; /*------*/  l++; // WARNING: UNOPTIMIZED FOR SHARED VERTICES
            }
        }
        cout << "\t\t\tTriangles: " << N_tri << ", indices: " << l << ", floats: " << k << endl;
        // per_tri_normals();
    }

    
    ///// Diagnostics ////////////////////////////

    void print_verts(){
        // Print the triangle data from the mesh
        ulong  k = 0;
        ushort l = 0;
        for( ulong i = 0; i < N_tri; i++ ){
            cout << "Tri " << i+1 << ": ";
            for( ubyte j = 0; j < 3; j++ ){
                cout << mesh.indices[l] << " {" << mesh.vertices[k+0] << ", " << mesh.vertices[k+1] << ", " << mesh.vertices[k+2] << "}, ";
                k += 3;
                l++;
            }
            cout << endl;
        }
    }


    ///// Constructors ///////////////////////////

    TriModel(){
        // Default constructor
        mesh = Mesh{};
    }

    TriModel( ulong Ntri ){
        mesh  = Mesh{};
        // mesh  = { 0 };
        N_tri = Ntri;
        N_vrt = Ntri * 3;
        // tDex  = 0;

        // Init geo memory
		mesh.triangleCount = N_tri;
    	mesh.vertexCount   = N_vrt;
        mesh.vertices /**/ = (float *)MemAlloc(N_vrt*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
        // mesh.normals /*-*/ = (float *)MemAlloc(N_vrt*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices /*-*/ = (ushort *)MemAlloc(N_vrt*sizeof(ushort));
        // mesh.vertices = new float[mesh.vertexCount * 3];
        // mesh.indices  = new unsigned short[mesh.vertexCount];

        // Init pose
		x = 0.0; // --- World X pos
		y = 0.0; // --- World Y pos
		z = 0.0; // --- World Z pos
		r = 0.0; // --- Local roll  angle
		p = 0.0; // --- Local pitch angle
		w = 0.0; // --- Local yaw   angle
        R = MatrixRotateXYZ( Vector3{ p, w, r } );
        T = Matrix{ R };
    }


    ///// Pose Math //////////////////////////////

    void set_XYZ( float x_, float y_, float z_ ){
		// Set the world XYZ position of the model
		x = x_;
		y = y_;
		z = z_;
	}

    void set_RPY( float r_, float p_, float y_ ){
		// Set the world Roll, Pitch, Yaw of the model
        r = r_; // --- Local roll  angle
		p = p_; // --- Local pitch angle
		w = y_; // --- Local yaw   angle
		R = MatrixRotateXYZ( Vector3{ p_, y_, r_ } );
        T = Matrix{ R }; 
        model.transform = T;
	}

    void rotate_RPY( float r_, float p_, float y_ ){
		r += r_;
		p += p_;
		w += y_;
		mx = MatrixRotateX( p );
		my = MatrixRotateY( w );
		mz = MatrixRotateZ( r );
		T  = MatrixMultiply( mx, my );
		T  = MatrixMultiply( mz, T  );
		T  = MatrixMultiply( T , R  );
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
        print_verts();
        cout << "\t\t\t`UploadMesh()` ..." << endl;
		UploadMesh( &mesh, true );
		// UploadMesh( &mesh, false );
        cout << "\t\t\t`LoadModelFromMesh()` ..." << endl;
    	model = LoadModelFromMesh( mesh );
	}

    void load_geo(){
        // Get the model ready for drawing
        cout << "\t\t`build_mesh()` ..." << endl;
        build_mesh();
        cout << "\t\t`load_mesh()` ..." << endl;
        load_mesh();
    }

    void draw(){
        cout << "Please implement `draw` for derived class!" << endl;
    };

};


////////// DELTA GLIDER ////////////////////////////////////////////////////////////////////////////

class DeltaGlider : public TriModel{
public:
    // A fun little space plane

    DeltaGlider( float wingspan = 10.0f ) : TriModel( 8 ){

        cout << "\tGlider init ..." << endl;

        float fusFrac   = 0.5;
        float sweptFrac = 0.75;
        float thickFrac = 0.25;

        cout << "\tAbout to load tris ..." << endl;

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

        cout << "\tCreate model ..." << endl;

        // TriModel( 8 );
        // cout << "\t`load_geo()` ..." << endl;
        // load_geo();
        cout << "\t`rotate_RPY()` ..." << endl;
        // rotate_RPY( 0.0, 3.1416/2.0, 0.0 );
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void draw(){
        // Draw the model
        DrawModel(model, Vector3{x, y, z}, 1.0, RED);  
        // DrawModelWires( model, Vector3{x, y, z}, 1.0, BLACK );
    }

};


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){

    cout << "About to create glider ..." << endl;

    // Ship
    DeltaGlider glider( 10.0f );
    cout << "About to set position ..." << endl;
    glider.set_XYZ( 0.0, 0.0, 2.0 );

    cout << "About to create camera ..." << endl;
    
    // Camera
    Camera camera = Camera{
        Vector3{ 12.0, 12.0, 12.0 }, // Position
        Vector3{ 0.0, 0.0, 2.0 }, // Target
        Vector3{  0.0, 0.0, 1.0 }, // Up
        45.0, // -------------------- FOV_y
        0 // ------------------------ Projection mode
    };

    cout << "About to start drawing ..." << endl;

    InitWindow(800, 450, "Spinning Ship");
    SetTargetFPS( 60 );
    // rlDisableBackfaceCulling(); 
    rlEnableSmoothLines();

    glider.load_geo(); // THIS MUST HAPPEN AFTER WINDOW INIT!

    while( !WindowShouldClose() ){
        
        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP //////////////////////////
        glider.rotate_RPY( 0.0, 3.1416/180.0, 0.0 );
        glider.draw();

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}

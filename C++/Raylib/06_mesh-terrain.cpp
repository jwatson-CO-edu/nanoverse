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

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

typedef unsigned long  ulong;
typedef unsigned short ushort;
typedef unsigned char  ubyte;

////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

float randf(){
    // Return a pseudo-random number between 0.0 and 1.0
    return  1.0f * rand() / RAND_MAX;
}

void rand_seed(){  srand( time(NULL) );  } // Seed RNG with unpredictable time-based seed


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

    // Model //
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
        N_tri = Ntri;
        N_vrt = Ntri * 3;

        // Init geo memory
		mesh.triangleCount = N_tri;
    	mesh.vertexCount   = N_vrt;
        mesh.vertices /**/ = (float *)MemAlloc(N_vrt*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
        mesh.indices /*-*/ = (ushort *)MemAlloc(N_vrt*sizeof(ushort));

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

    TerrainPlate( float scale = 10.0f, ulong Mrows = 10, ulong Ncols = 10 ){
        // Generate points and load triangles
        
        // 0. Init
        M /**/ = Mrows;
        N /**/ = Ncols;
        scl    = scale;
        offset = scale/50.0f;
        gndClr = GREEN;
        linClr = BLACK;

        // 1. Generate points
        vector<Vector3> row;
        for( ulong i = 0; i < M; i++ ){
            row.clear();
            for( ulong j = 0; j < N; j++ ){
                row.push_back(  Vector3{ j*scl, i*scl, randf()*scl }  );
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
                    load_tri( v1, v4, v2 );
                    load_tri( v1, v3, v2 );
                }else{
                    load_tri( v1, v3, v2 );
                    load_tri( v2, v3, v4 );
                }
            }
        }
    }

    void draw(){
        
    }
};
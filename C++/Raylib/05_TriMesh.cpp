// gcc 03_simple-terrain.cpp -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <array>     
using std::array;  
#include <vector>     
using std::vector;  
#include <iostream>
using std::cout, std::endl;

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h" // `rlDisableBackfaceCulling`

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
    ulong /*--------------*/ tDex; //- Index offset for triangle coords

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
        tDex++;
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
    }

    void load_mesh(){
		// Send triangle mesh geometry to RayLib, needed for drawing
		UploadMesh( &mesh, true );
    	model = LoadModelFromMesh( mesh );
	}

    void load_geo(){
        // Get the model ready for drawing
        build_mesh();
        load_mesh();
    }
    

    ///// Constructors ///////////////////////////

    TriModel(){
        // Default constructor
        mesh = Mesh{};
    }

    TriModel( ulong Ntri ){
        N_tri = Ntri;
        N_vrt = Ntri * 3;
        mesh = Mesh{};
        tDex = 0;

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

    void draw(){
        cout << "Please implement `draw` for derived class!" << endl;
    };

};


////////// DELTA GLIDER ////////////////////////////////////////////////////////////////////////////

class DeltaGlider : public TriModel{
    // A fun little space plane

    DeltaGlider( float wingspan = 10.0f ){
        float fusFrac   = 0.5;
        float sweptFrac = 0.75;
        float thickFrac = 0.25;

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

        TriModel( 8 );
        load_geo();
    }

    void draw(){
        // FIXME, START HERE: DRAW A WIREFRAME MODEL
    }

};


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////
int main(){
    return 0;
}

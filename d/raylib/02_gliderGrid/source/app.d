// Adapted from: https://www.reddit.com/r/raylib/comments/v2su1s/help_with_dynamic_mesh_creation_in_raylib/

import core.stdc.stdlib; // `malloc`
import raylib; // --------- easy graphics

class TriMesh{
	/// Members ///
	int   N_tri; // Number of triangles 
	ulong vDex;
	ulong iDex;
	Mesh  mesh; //- Raylib mesh geometry
	Model model; // Raylib drawable model

	/// Constructors ///
	this( int n ){
		// Init mesh
		vDex  = 0;
		iDex  = 0;
		N_tri = n;
		mesh  = Mesh();
		
		// Init geo memory
		mesh.triangleCount = n;
    	mesh.vertexCount   = n * 3;
		mesh.vertices      = cast(float*)malloc(float.sizeof * mesh.vertexCount * 3);
    	mesh.indices       = cast(ushort*)malloc(ushort.sizeof * mesh.vertexCount);
	}

	public bool push_vertex( float x, float y, float z ){
		if(vDex+3 > mesh.vertexCount*3){  return false;  }else{
			mesh.vertices[vDex + 0] = x;
			mesh.vertices[vDex + 1] = y;
			mesh.vertices[vDex + 2] = z;
			vDex += 3;
			return true;
		}
	}

	public bool push_triangle( ushort p1, ushort p2, ushort p3 ){
		if(iDex+3 > mesh.vertexCount){  return false;  }else{
			mesh.indices[iDex + 0] = p1;
			mesh.indices[iDex + 1] = p2;
			mesh.indices[iDex + 2] = p3;
			iDex += 3;
			return true;
		}
	}
}

void main()
{
	// // call this before using raylib
	// validateRaylibBinding();
	
    // // Window / Display Params
    // InitWindow(800, 600, "Hello, Raylib-D!");
    // SetTargetFPS(60);
    // rlDisableBackfaceCulling();
    // rlEnableSmoothLines();

    // // Camera
    // Camera camera = Camera(
    //     Vector3(5.0, 5.0, 5.0), 
    //     Vector3(0.0, 0.0, 0.0), 
    //     Vector3(0.0, 1.0, 0.0), 
    //     45.0, 
    //     0
    // );

    // // Entities
    // int n = 1;
    // Mesh mesh = Mesh();
    // mesh.triangleCount = n;
    // mesh.vertexCount   = n * 3;
    // mesh.vertices      = cast(float*)malloc(float.sizeof * mesh.vertexCount * 3);
    // mesh.indices       = cast(ushort*)malloc(ushort.sizeof * mesh.vertexCount);
    
    // mesh.vertices[0]   = 0.0;
    // mesh.vertices[1]   = 0.0;
    // mesh.vertices[2]   = 0.0;
    
    // mesh.vertices[3]   = 0.0;
    // mesh.vertices[4]   = 0.0;
    // mesh.vertices[5]   = 1.0;
    
    // mesh.vertices[6]   = 0.0;
    // mesh.vertices[7]   = 1.0;
    // mesh.vertices[8]   = 0.0;
    
    // mesh.indices[0]    = 0;
    // mesh.indices[1]    = 1;
    // mesh.indices[2]    = 2;

    // UploadMesh(&mesh, true);
    // Model model = LoadModelFromMesh(mesh);

    // while (!WindowShouldClose())
	// {
	// 	BeginDrawing();
	// 	ClearBackground(Colors.RAYWHITE);

    //     BeginMode3D(camera);
    //     DrawModelWires(model, Vector3(0.0, 0.0, 0.0), 1.00, Colors.RED);
    //     DrawGrid(10, 1.0);
    //     EndMode3D();

	// 	DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
	// 	EndDrawing();
	// }
	// CloseWindow();
}
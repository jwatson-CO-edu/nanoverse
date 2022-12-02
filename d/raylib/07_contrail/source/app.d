////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import core.stdc.stdlib; // `malloc`
import raylib; // --------- easy graphics
import std.stdio; // ------ writeline

import rlights;
import toybox;


////////// CONTRAIL ////////////////////////////////////////////////////////////////////////////////

class FlexMesh{
	// Mesh with easier access to underlying points
	int   N_tri; // Number of triangles 
	Mesh  mesh; //- Raylib mesh geometry
	Model model; // Raylib drawable model

	/// Constructors ///
	this( int n ){
		N_tri = n;
		mesh  = Mesh();
		
		// Init geo memory
		mesh.triangleCount = n;
    	mesh.vertexCount   = n * 3;
		// vertices /*-----*/ = cast(float* ) new float[  mesh.vertexCount*3 ];
    	// indices /*------*/ = cast(ushort*) new ushort[ mesh.vertexCount   ];
		// colors /*-------*/ = cast(ubyte* ) new ubyte[  mesh.vertexCount*4 ];
		mesh.vertices /**/ = cast(float* ) malloc( float.sizeof  * mesh.vertexCount * 3);
    	mesh.indices /*-*/ = cast(ushort*) malloc( ushort.sizeof * mesh.vertexCount    );
		mesh.colors /*--*/ = cast(ubyte* ) malloc( ubyte.sizeof  * mesh.vertexCount * 4);
	}

	bool assign_vertices( ushort len, float[] vertData, ushort bgn = 0 ){
		if( (len+bgn-1) < (mesh.vertexCount*3) ){
			for( ushort i = bgn; i < len+bgn; i++ ){
				mesh.vertices[i] = vertData[i-bgn];
			}
			return true;
		}else  return false;
	}

	bool assign_indices( ushort len, ushort[] dexData, ushort bgn = 0 ){
		if( (len+bgn-1) < (mesh.vertexCount) ){
			for( ushort i = bgn; i < len+bgn; i++ ){
				mesh.indices[i] = dexData[i-bgn];
			}
			return true;
		}else  return false;
	}

	bool assign_colors( ushort len, ushort[] clrData, ushort bgn = 0 ){
		if( (len+bgn-1) < (mesh.vertexCount*4) ){
			for( ushort i = bgn; i < len+bgn; i++ ){
				mesh.colors[i] = clrData[i-bgn];
			}
			return true;
		}else  return false;
	}

	public void load_geo(){
		// Send triangle mesh geometry to RayLib, needed for drawing
		UploadMesh(&mesh, true);
    	model = LoadModelFromMesh( mesh );
	}

}




////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

class FlightThirdP_Camera{
	// Camera control for a 3rd person view of an aircraft/spacecraft
	// 2022-09-22: Right now fixing the camera to the ship by a rigid "stick" 
	//             without obstacle avoidance or damping

	Camera  camera; // --- The actual RayLib camera
	Vector3 trgtCenter; // Position of the target
	Matrix  trgtXform; //- Orientation of the target
	Vector3 offset_t; // - Desired camera offset in the target's frame

	this( Vector3 desiredOffset_V3, Vector3 tCenter, Matrix tXform ){
		trgtCenter = tCenter; // Position of the target
		trgtXform  = tXform; // -------- Orientation of the target
		offset_t   = desiredOffset_V3; // --------- Desired camera offset in the target's frame
		camera = Camera(
			Vector3Add( tCenter, Vector3Transform( offset_t, trgtXform) ), // Camera location, world frame
			tCenter, // ----------------------------------------------------- Camera target, world frame
			Vector3Transform( Vector3(0.0, 0.0, 1.0), trgtXform), // -------- Up vector
			45.0, 
			0
		);
	}

	public void update_target_position( Vector3 tCenter ){  trgtCenter = tCenter;  }
	
	public void update_target_orientation( Matrix tXform ){  trgtXform = tXform;  }

	public void advance_camera(){
		// Move the camera after all the target updates are in
		camera.position = Vector3Add( trgtCenter, Vector3Transform( offset_t, trgtXform) );
		camera.target   = trgtCenter;
		camera.up       =  Vector3Transform( Vector3(0.0, 0.0, 1.0), trgtXform);
		// camera.Update();
	}

	public void begin(){  BeginMode3D(camera);  }
	public void end(){  EndMode3D();  }
	
}



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

// uint MAX_LIGHTS = 4;

void main(){
	// call this before using raylib
	validateRaylibBinding();
	
    // Window / Display Params
    InitWindow(800, 600, "Hello, Raylib-D!");
    SetTargetFPS(60);
    rlDisableBackfaceCulling();
    rlEnableSmoothLines();

	

	// FrameAxes worldFrame = new FrameAxes();
	GridXY    worldGrid  = new GridXY( 
		Vector3(0.0, 0.0, 0.0), 
		  1.0, 
		500, // https://github.com/raysan5/raylib/issues/1051#issue-543279679
		Colors.GRAY, // https://robloach.github.io/raylib-cpp/classraylib_1_1_color.html
		false
	);
	GridXY    skyGrid  = new GridXY( 
		Vector3(0.0, 0.0, 200.0), 
		  1.0, 
		500, // https://github.com/raysan5/raylib/issues/1051#issue-543279679
		Colors.BLUE, // https://robloach.github.io/raylib-cpp/classraylib_1_1_color.html
		false
	);

	DeltaShip glider = new DeltaShip( 2.0 );
	glider.rotate_RPY( 0.0, 3.1416/2.0, 0.0 );
	glider.set_XYZ(0.0, 0.0, 2.0);
	// glider.set_RPY( 0.0, 0.0, 0.0 ); // 3.1416/2.0

	// Shaders
	// https://www.raylib.com/examples/shaders/loader.html?name=shaders_basic_lighting
	// Load basic lighting shader
	Shader shader = LoadShader("source/lighting.vs", "source/lighting.fs");
	// Get some required shader locations
	shader.locs[ShaderLocationIndex.SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
	// Ambient light level (some basic lighting)
    int ambientLoc = GetShaderLocation(shader, "ambient");
	float[4] arr = [0.1f, 0.1f, 0.1f, 1.0f];
	SetShaderValue( 
		shader, ambientLoc, arr.ptr, ShaderUniformDataType.SHADER_UNIFORM_VEC4
	); //                   ^^^^^^^---void* here
	// Assign our lighting shader to model
	glider.model.materials[0].shader = shader;
	// Create lights
    Light[LIGHT_CONST.MAX_LIGHTS] lights; // = { 0 };
    lights[0] = CreateLight( LightType.LIGHT_POINT, Vector3( -2, 1, -2 ), Vector3Zero(), Colors.YELLOW, shader);
    lights[1] = CreateLight( LightType.LIGHT_POINT, Vector3( 2, 1, 2 ), Vector3Zero(), Colors.RED, shader);
    lights[2] = CreateLight( LightType.LIGHT_POINT, Vector3( -2, 1, 2 ), Vector3Zero(), Colors.GREEN, shader);
    lights[3] = CreateLight( LightType.LIGHT_POINT, Vector3( 2, 1, -2 ), Vector3Zero(), Colors.BLUE, shader);

	FlightThirdP_Camera camera = new FlightThirdP_Camera(
		Vector3( 0.0,  2.5, -6.0 ),
		glider.get_XYZ(),
		glider.T
	);

	while (!WindowShouldClose()){

		/// Update ///
		if( IsKeyDown( KeyboardKey.KEY_LEFT  ) ){  glider.rotate_RPY( 0.0,  0.0,          3.1416/120.0 );  }
		if( IsKeyDown( KeyboardKey.KEY_RIGHT ) ){  glider.rotate_RPY( 0.0,  0.0,         -3.1416/120.0 );  }
		if( IsKeyDown( KeyboardKey.KEY_UP    ) ){  glider.rotate_RPY( 0.0,  3.1416/120.0, 0.0          );  }
		if( IsKeyDown( KeyboardKey.KEY_DOWN  ) ){  glider.rotate_RPY( 0.0, -3.1416/120.0, 0.0          );  }

		camera.update_target_position( glider.get_XYZ() );
		camera.update_target_orientation( glider.T );
		camera.advance_camera();

		/// Rendering ///

		BeginDrawing();
		ClearBackground(Colors.RAYWHITE);

		
        camera.begin();
		

		glider.z_thrust( 2.0/60.0 );
		glider.draw();

		// worldFrame.draw();
		worldGrid.draw();
		// skyGrid.draw();

		// DrawGrid(10, 1.0);
        
		camera.end();

		// DrawText("Hello, World!", 400, 300, 28, Colors.BLACK);
		EndDrawing();
	}

	CloseWindow();
}
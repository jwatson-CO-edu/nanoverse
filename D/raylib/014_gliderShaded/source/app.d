////////// INIT ////////////////////////////////////////////////////////////////////////////////////

import core.stdc.stdlib; // `malloc`
import raylib; // --------- easy graphics
import std.stdio; // ------ writeline

import rlights;


////////// GRAPHICS CLASSES ////////////////////////////////////////////////////////////////////////

///// Models / Meshes ////////////////////////////

class TriMesh{
	/// Members ///
	int   N_tri; // Number of triangles 
	ulong vDex; //- Index offset for vertex coords
	ulong iDex; //- Index offset for face index
	Mesh  mesh; //- Raylib mesh geometry
	Model model; // Raylib drawable model
	float x; // --- World X pos
	float y; // --- World Y pos
	float z; // --- World Z pos
	Matrix R; // -- World rotation
	float r; // --- Local roll  angle
	float p; // --- Local pitch angle
	float w; // --- Local yaw   angle
	Matrix mx; // - Local pitch
	Matrix my; // - Local yaw
	Matrix mz; // - Local roll
	Matrix T; // -- World orientation

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

		// Init pose
		x = 0.0; // ------------ World X pos
		y = 0.0; // ------------ World Y pos
		z = 0.0; // ------------ World Z pos
		r = 0.0; // ------------ Local roll  angle
		p = 0.0; // ------------ Local pitch angle
		w = 0.0; // ------------ Local yaw   angle
		mx = MatrixIdentity(); // - Local pitch
		my = MatrixIdentity(); // - Local yaw
		mz = MatrixIdentity(); // - Local roll
		R  = MatrixIdentity(); // Init identity
		T  = MatrixIdentity(); // Init identity
	}

	/// Methods ///

	public bool push_vertex( float x, float y, float z ){
		// Add a vertex
		if(vDex+3 > mesh.vertexCount*3){  return false;  }else{
			mesh.vertices[vDex + 0] = x;
			mesh.vertices[vDex + 1] = y;
			mesh.vertices[vDex + 2] = z;
			vDex += 3;
			return true;
		}
	}

	public bool push_triangle( ushort p1, ushort p2, ushort p3 ){
		// Add a triangle in CCW order spanning 3 vertex indices
		if(iDex+3 > mesh.vertexCount){  return false;  }else{
			mesh.indices[iDex + 0] = p1;
			mesh.indices[iDex + 1] = p2;
			mesh.indices[iDex + 2] = p3;
			iDex += 3;
			return true;
		}
	}

	public void load_geo(){
		// Send triangle mesh geometry to RayLib, needed for drawing
		UploadMesh(&mesh, true);
    	model = LoadModelFromMesh(mesh);
		// writeln( "geo loaded" );
	}

	public Vector3 get_XYZ(){  return Vector3( x, y, z );  }

	public void set_XYZ( float x_, float y_, float z_ ){
		// Set the world XYZ position of the model
		x = x_;
		y = y_;
		z = z_;
	}

	public void set_RPY( float r_, float p_, float y_ ){
		// Set the world Roll, Pitch, Yaw of the model
		R = MatrixRotateXYZ( Vector3( p_, y_, r_ ) );
	}

	public void rotate_RPY( float r_, float p_, float y_ ){
		r += r_;
		p += p_;
		w += y_;
		mx = MatrixRotateX( p );
		my = MatrixRotateY( r );
		mz = MatrixRotateZ( w );
		T  = MatrixMultiply( mz, my );
		T  = MatrixMultiply( mx, T  );
		T  = MatrixMultiply( T, R );
		// writeln( T );
	}

	public void z_thrust( float d = 0.0 ){
		// Advance a plane model in the forward direction (local Z)
		Vector3 vec = Vector3Transform(Vector3( 0.0, 0.0, d ), T);
		x += vec.x;
		y += vec.y;
		z += vec.z;
	}

	public void draw(){
		// Draw the model at the current pose
		model.transform = T;
        // DrawModelWires(model, Vector3(x, y, z), 1.0, Colors.RED);
        DrawModel(model, Vector3(x, y, z), 1.0, Colors.RED); // What happens when you call this with color data populated?
	}
}


class DeltaShip:TriMesh{
	// A cool and simple starship / glider 

	this( float wingspan = 10.0, float fusFrac = 0.5, float sweptFrac = 0.75, float thickFrac = 0.25 ){
		// TriMesh constructor
		super( 8 );
		// Verts
		super.push_vertex(  0.0        ,  0.0                  , 0.0                 + wingspan*sweptFrac/2.0 ); // 0, Front
		super.push_vertex(  0.0        , +wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0 ); // 1, Top peak
		super.push_vertex(  0.0        , -wingspan*thickFrac/2 , -wingspan*fusFrac/2 + wingspan*sweptFrac/2.0 ); // 2, Bottom peak
		super.push_vertex(  0.0        ,  0.0                  , -wingspan*fusFrac   + wingspan*sweptFrac/2.0 ); // 3, Back
		super.push_vertex( -wingspan/2 ,  0.0                  , -wingspan*sweptFrac + wingspan*sweptFrac/2.0 ); // 4, Left wingtip
		super.push_vertex( +wingspan/2 ,  0.0                  , -wingspan*sweptFrac + wingspan*sweptFrac/2.0 ); // 5, Right wingtip
		// Faces
		super.push_triangle( 0,4,1 ); // Left  Top    Front
		super.push_triangle( 1,4,3 ); // Left  Top    Back 
		super.push_triangle( 5,0,1 ); // Right Top    Front
		super.push_triangle( 5,1,3 ); // Right Top    Back 
		super.push_triangle( 0,2,4 ); // Left  Bottom Front
		super.push_triangle( 2,3,4 ); // Left  Bottom Back
		super.push_triangle( 0,5,2 ); // Right Bottom Front
		super.push_triangle( 2,5,3 ); // Right Bottom Back
		// load
		super.load_geo();
	}
}



///// Line Segments //////////////////////////////

class FrameAxes{
	// Classic XYX-RGB axes
	float   len; // -- Length of basis vectors
	Vector3 origin; // World position
	Matrix  T; // ---- World orientation

	this(){
		len    = 1.0;
		origin = Vector3( 0.0, 0.0, 0.0 );
		T      = MatrixIdentity();
	}

	this( float unitLen, Vector3 orig, Matrix transform ){
		len    = unitLen;
		origin = orig;
		T      = transform;
	}

	this( float unitLen, Vector3 orig ){
		len    = unitLen;
		origin = orig;
		T      = MatrixIdentity();
	}

	public void draw(){
		// Render classic coordinate axes
		DrawLine3D( // X basis
			origin, 
			Vector3Add( origin, Vector3Transform( Vector3( len, 0.0, 0.0 ), T)  ),
			Colors.RED
		);
		DrawLine3D( // Y basis
			origin, 
			Vector3Add( origin, Vector3Transform( Vector3( 0.0, len, 0.0 ), T)  ),
			Colors.GREEN
		);
		DrawLine3D( // Z basis
			origin, 
			Vector3Add( origin, Vector3Transform( Vector3( 0.0, 0.0, len ), T)  ),
			Colors.BLUE
		);
	}
}

class GridXY{
	// XY grid to replace the stock XZ grid (Z is UP!)
	Vector3   origin; // ----- Location of grid center
	float     unitLen; // ---- Dimension of grid unit
	uint      oneSideLen_u; // Extent of grid in each direction +/-, in units
	Color     color; // ------ Color of grid lines
	bool      drawAxes; // --- Whether or not to draw axes
	FrameAxes frame; // ------ Optional coordinate frame

	this(){
		origin       = Vector3( 0.0, 0.0, 0.0 ); // Location of grid center
		unitLen      =  1.0; // ------------------- Dimension of grid unit
		oneSideLen_u = 10; // --------------------- Extent of grid in each direction +/-, in units
		color        = Colors.BLACK; // ----------- Color of grid lines
		drawAxes     = true; // ------------------- Whether or not to draw axes
		frame        = new FrameAxes( unitLen, Vector3Add( origin, Vector3( 0.0, 0.0, unitLen ) ) ); // -------- Optional coordinate frame
	}

	this( Vector3 orig, float unit = 1.0, uint oneSide_u = 10, Color clr = Colors.BLACK, bool axes = true ){
		origin       = orig; // Location of grid center
		unitLen      = unit; // ------------------- Dimension of grid unit
		oneSideLen_u = oneSide_u; // --------------------- Extent of grid in each direction +/-, in units
		color        = clr; // ----------- Color of grid lines
		drawAxes     = axes; // ------------------- Whether or not to draw axes
		if( drawAxes ){
			frame = new FrameAxes( unitLen, Vector3Add( origin, Vector3( 0.0, 0.0, unitLen ) ) );
		}
	}

	public void draw(){
		// Render an XY grid, with optional coordinate axes (frame debugging)
		float dHalf = unitLen * oneSideLen_u;
		// Draw the center cross
		DrawLine3D( 
			Vector3Add( origin, Vector3( -dHalf, 0.0, 0.0 )  ),
			Vector3Add( origin, Vector3(  dHalf, 0.0, 0.0 )  ),
			color
		);
		DrawLine3D( 
			Vector3Add( origin, Vector3( 0.0, -dHalf, 0.0 )  ),
			Vector3Add( origin, Vector3( 0.0,  dHalf, 0.0 )  ),
			color
		);
		// Draw X and Y lines, double-sided
		for (uint i = 1; i < oneSideLen_u; i++){

			DrawLine3D( 
				Vector3Add( origin, Vector3( -dHalf,  unitLen*i, 0.0 )  ),
				Vector3Add( origin, Vector3(  dHalf,  unitLen*i, 0.0 )  ),
				color
			);
			DrawLine3D( 
				Vector3Add( origin, Vector3( -dHalf, -unitLen*i, 0.0 )  ),
				Vector3Add( origin, Vector3(  dHalf, -unitLen*i, 0.0 )  ),
				color
			);

			DrawLine3D( 
				Vector3Add( origin, Vector3( unitLen*i, -dHalf, 0.0 )  ),
				Vector3Add( origin, Vector3( unitLen*i,  dHalf, 0.0 )  ),
				color
			);
			DrawLine3D( 
				Vector3Add( origin, Vector3( -unitLen*i, -dHalf, 0.0 )  ),
				Vector3Add( origin, Vector3( -unitLen*i,  dHalf, 0.0 )  ),
				color
			);
		}
		// Draw optional axes
		if( drawAxes ){  frame.draw();  }
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
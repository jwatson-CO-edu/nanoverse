import std.stdio;
import std.math;

import raylib; // --------- easy graphics
import raylib.raymath; // --------- easy graphics

alias pi = raylib.PI;



////////// GAME COMPONENTS /////////////////////////////////////////////////////////////////////////

const float EDGE_LEN = 75.0f;

enum SC_Structure{
	// Structure types
	ROAD,
	TOWN,
	CITY,
}

enum SC_Player{
	// Player colors
	RED, 
	ORN, 
	WHT, 
	BLU,  
}

enum SC_Resource{
	// Resource types
	BRCKS,
	LUMBR,
	ROCKS,
	GRAIN,
	SHEEP,
	WLDCD // Wild card
}

struct StructSC{
	// Player structure container
	SC_Player    owner;
	SC_Structure type;
}

struct EdgeSC{
	// Edge container, holds roads
	StructSC   road;
	NodeSC*[2] ends;
}

struct LandSC{
	// Land container
	Vector2     location; //- Location in 2D space
	ushort /**/ yieldRoll; // Dice rolls that yields the resource
	SC_Resource type; // ---- Type of land this is
	Color /*-*/ color; // --- Display color
	NodeSC*[]   nodes; // --- Nodes that this land supplies on a yield

	void draw(){
		DrawPoly( location, 6, EDGE_LEN, 0.0, color);
	}
}

struct NodeSC{
	// Board vertex container
	Vector2    loc; // Location in 2D space
	bool /*-*/ opn; // Is this node open for building?
	StructSC   con; // Structure contents
	GreyManSC* vst; // Pointer to The Grey Man, when present
	EdgeSC*[]  edg; // Edges connected to node

}

class GreyManSC{
	// The Grey Man, possibly a robber?
	SC_Resource[] hand; // Resources held
}

class PlayerSC{
	// Models one of the game players
	SC_Resource[] hand; // Resources held
	ushort /*--*/ VP; // - Victory points
}

struct PortSC{
	// Exchange port
	NodeSC*[]     nodes; // --- Nodes that this port supplies
	SC_Resource[] takes;
	SC_Resource[] gives;
}

////////// GAME BOARD //////////////////////////////////////////////////////////////////////////////

Vector2[6]  REL_NODE_LOCS;
Vector2[6]  TILE_CENTERS;

struct BoardSC{
	// Container for everything on the game board
	SC_Resource[] bank;  // Total unowned resources
	LandSC*[]     hexes; // Land tiles
	NodeSC*[]     nodes; // Land vertices
	EdgeSC*[]     edges; // Segments between nodes, possibly holding roads

	bool add_tile(){
		// FIXME, START HERE: MAKE SURE NODES AND EDGES ARE PROPERLY CONNECTED BY DISTANCE
		return true;
	}

	void draw_tiles(){

	}

	void draw_nodes(){

	}

	void draw_edges(){

	}
}


void main(){

	///// Setup //////////////////////////////////
	
	TILE_CENTERS = [
		
		// Row 1
		Vector2( 400, 200 ),
		Vector2( 400 + 2*EDGE_LEN*cos(pi/6.0), 200 ),
		Vector2( 400 + 4*EDGE_LEN*cos(pi/6.0), 200 ),
		
		// Row 2
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0), 
			200 + 3*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 2*EDGE_LEN*cos(pi/6.0), 
			200 + 3*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 4*EDGE_LEN*cos(pi/6.0), 
			200 + 3*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 6*EDGE_LEN*cos(pi/6.0), 
			200 + 3*EDGE_LEN*cos(pi/3.0) 
		),

		// Row 3

	];

	LandSC l1 = LandSC(
		TILE_CENTERS[0], //- Location in 2D space
		6, // Dice rolls that yields the resource
		SC_Resource.ROCKS, // ---- Type of land this is
		Colors.GRAY // --- Display color
	);

	LandSC l2 = LandSC(
		TILE_CENTERS[3], //- Location in 2D space
		6, // Dice rolls that yields the resource
		SC_Resource.SHEEP, // ---- Type of land this is
		Colors.GREEN // --- Display color
	);

	REL_NODE_LOCS = [ // Locations of nodes relative to a land tile center
		Vector2( 0.0f, EDGE_LEN ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -1.0f*pi/3.0f ),
		// Vector2( 0.0f, EDGE_LEN ).Rotate(  -1.0*PI/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -2.0*pi/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -3.0*pi/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -4.0*pi/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -5.0*pi/3.0 ),
	];

	///// Rendering //////////////////////////////

	// call this before using raylib
	validateRaylibBinding();

	// Window / Display Params
    InitWindow(1200, 900, "Hello, Raylib-D!");
    SetTargetFPS(60);

	writeln( "##### Settlers of C*t*n Simulation, Version 2022-12 #####" );


	while ( !WindowShouldClose() ){

		/// Rendering ///
		BeginDrawing();

		ClearBackground( Colors.DARKBLUE );

		l1.draw();
		l2.draw();
		
		EndDrawing();

	}

	CloseWindow();
}

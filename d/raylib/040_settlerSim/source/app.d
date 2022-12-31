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

Vector2[ 6] REL_NODE_LOCS; // Node locations relative to the tile center
Vector2[19] TILE_CENTERS; //- Tile center locations, absolute coordinates

struct BoardSC{
	// Container for everything on the game board

	/// Members ///
	SC_Resource[] bank;  // --------- Total unowned resources
	LandSC*[]     hexes; // --------- Land tiles
	NodeSC*[]     nodes; // --------- Land vertices
	EdgeSC*[]     edges; // --------- Segments between nodes, possibly holding roads

	/// Methods ///
	
	bool add_tile( LandSC* nuTile ){
		// Add a tile to the game board and make the appropriate connections
		Vector2 searchLoc; // ----- Potential location for a node
		bool    foundNode = false; // Did a linear search return a hit?
		NodeSC* nuNode    = null; // New node to add
		NodeSC* lastNode  = null; // Other end of a connection
		EdgeSC* nuEdge    = null; // New edge to add

		// If the (4-player) board is not full, then proceed with adding a tile
		if( hexes.length < TILE_CENTERS.length ){

			// Add the tile
			hexes ~= nuTile;

			// For each of the possible node locations
			foreach( Vector2 rel; REL_NODE_LOCS ){

				foundNode = false;

				// Get the absolute node location
				searchLoc = Vector2Add( nuTile.location, rel );
				
				// Search for an existing node
				foreach( NodeSC* node; nodes ){
					// Check the distance between the prospective node and the existing node
					if( Vector2DistanceSqr( node.loc, searchLoc ) < (EDGE_LEN/2.0f) ){
						// If node already exists, then add a pointer to it
						nuTile.nodes ~= node;
						foundNode = true;
						break;
					}
				}
				// If node DNE, then create and set pointer to it
				if( !foundNode ){
					nuNode = new NodeSC( searchLoc, true );
					nuTile.nodes ~= nuNode; // Add to tile nodes
					nodes ~= nuNode; // Add to node collection
				}
			}

			// For each of the nodes attached to the tile
			ushort i = 0;
			lastNode = nuTile.nodes[$-1];
			foreach( NodeSC* node; nuTile.nodes ){ // Assume that the nodes were added in clockwise order, above
				foundNode = false;
				// Check that there isn't already a connection between this node and last
				foreach( EdgeSC* edge; node.edg ){
					if( ((edge.ends[0] == lastNode) && (edge.ends[1] == node)) 
						|| 
						((edge.ends[1] == lastNode) && (edge.ends[0] == node)) ){
						foundNode = true;
						break;
					}
				}
				// If there is not a connection, then add an edge
				if( !foundNode ){

				}
				lastNode = node;
			}

			// Report success for tile add
			return true;
		}
		// Else the board was full, report failure for tile add
		return false;
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
		Vector2( 
			400 - 2*EDGE_LEN*cos(pi/6.0), 
			200 + 6*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - 2*EDGE_LEN*cos(pi/6.0) + 2*EDGE_LEN*cos(pi/6.0),
			200 + 6*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - 2*EDGE_LEN*cos(pi/6.0) + 4*EDGE_LEN*cos(pi/6.0),
			200 + 6*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - 2*EDGE_LEN*cos(pi/6.0) + 6*EDGE_LEN*cos(pi/6.0),
			200 + 6*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - 2*EDGE_LEN*cos(pi/6.0) + 8*EDGE_LEN*cos(pi/6.0),
			200 + 6*EDGE_LEN*cos(pi/3.0) 
		),

		// Row 4
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0), 
			200 + 8*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 2*EDGE_LEN*cos(pi/6.0), 
			200 + 8*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 4*EDGE_LEN*cos(pi/6.0), 
			200 + 8*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 6*EDGE_LEN*cos(pi/6.0), 
			200 + 8*EDGE_LEN*cos(pi/3.0) 
		),

		// Row 5
		Vector2( 
			400, 
			200 + 10*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 + 2*EDGE_LEN*cos(pi/6.0), 
			200 + 10*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 + 4*EDGE_LEN*cos(pi/6.0), 
			200 + 10*EDGE_LEN*cos(pi/3.0) 
		),
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

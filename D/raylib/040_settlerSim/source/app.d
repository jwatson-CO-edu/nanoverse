import std.stdio;
import std.math;
import std.conv;
import std.string : toStringz;

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

Color[SC_Player] PLAYER_COLORS;

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
	NodeSC*[2] ends;
	StructSC   road;

	void draw(){
		DrawLineEx( ends[0].loc, ends[1].loc, 2, Colors.BLACK); 
	}
}

struct LandSC{
	// Land container
	Vector2 /*-*/ location; //- Location in 2D space
	ushort /*--*/ yieldRoll; // Dice rolls that yields the resource
	SC_Resource   type; // ---- Type of land this is
	Color /*---*/ color; // --- Display color
	NodeSC*[]     nodes; // --- Nodes that this land supplies on a yield
	ushort /*--*/ id; // ------ ID for this particular node 
	static ushort ID = 0; // -- Total number of IDs created

	this( Vector2 loc, ushort roll, SC_Resource typ, Color clr ){
		// Create the node, assign an ID, then increment created IDs
		location  = loc; //- Location in 2D space
		yieldRoll = roll; // Dice rolls that yields the resource
		type /**/ = typ; //- Type of land this is
		color     = clr; //- Display color
		id /*--*/ = ID;
		ID++;
	}

	void draw(){
		DrawPoly( location, 6, EDGE_LEN, 0.0, color);
		DrawText( (yieldRoll.to!string).toStringz(), cast(int) location.x, cast(int) location.y, 40, Colors.BLACK );
	}
}

struct NodeSC{
	// Board vertex container
	Vector2 /*-*/ loc; // -- Location in 2D space
	bool /*----*/ opn; // -- Is this node open for building?
	StructSC /**/ con; // -- Structure contents
	GreyManSC*    vst; // -- Pointer to The Grey Man, when present
	EdgeSC*[]     edg; // -- Edges connected to node
	ushort /*--*/ id; // --- ID for this particular node 
	static ushort ID = 0; // Total number of IDs created
	
	this( Vector2 location ){
		// Create the node, assign an ID, then increment created IDs
		loc = location;
		opn = true;
		id  = ID;
		ID++;
	}

	void draw(){
		DrawCircle( cast(int) loc.x, cast(int) loc.y, EDGE_LEN/8.0, Colors.BLACK);
	}
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
	SC_Resource[]   bank;  // --------- Total unowned resources
	LandSC*[] /*-*/ hexes; // --------- Land tiles
	LandSC*[ushort] hexLookup; // ----- Find a hex by its ID
	NodeSC*[] /*-*/ nodes; // --------- Land vertices
	EdgeSC*[] /*-*/ edges; // --------- Segments between nodes, possibly holding roads

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

			// Add the tile to the board and the lookup
			hexes ~= nuTile;
			hexLookup[ nuTile.id ] = nuTile; 

			// For each of the possible node locations
			foreach( Vector2 rel; REL_NODE_LOCS ){

				foundNode = false;

				// Get the absolute node location
				searchLoc = Vector2Add( nuTile.location, rel );
				writeln( "Find or create a node at " ~ searchLoc.to!string ~ " = " ~ 
				 		 nuTile.location.to!string ~ " + " ~ rel.to!string );
				
				// Search for an existing node
				foreach( NodeSC* node; nodes ){
					// Check the distance between the prospective node and the existing node
					// if( Vector2DistanceSqr( node.loc, searchLoc ) < (EDGE_LEN/4.0f) ){
					if( Vector2DistanceSqr( node.loc, searchLoc ) < (EDGE_LEN) ){
						// If node already exists, then add a pointer to it
						nuTile.nodes ~= node;
						foundNode = true;
						writeln( "Found a MATCH at " ~ node.loc.to!string );
						break;
					}
				}
				// If node DNE, then create and set pointer to it
				if( !foundNode ){
					nuNode = new NodeSC( searchLoc );
					nuTile.nodes ~= nuNode; // Add to tile nodes
					nodes ~= nuNode; // Add to node collection
				}
			}

			// For each of the nodes attached to the tile
			// ushort i = 0;
			lastNode = nuTile.nodes[$-1];
			foreach( NodeSC* node; nuTile.nodes ){ // Assume that the nodes were added in clockwise order, above
				writeln( "Edge [ " ~ lastNode.id.to!string ~ ", " ~ node.id.to!string ~ " ]"  );
				foundNode = false;
				// Check that there isn't already a connection between this node and last
				// foreach( EdgeSC* edge; node.edg ){
				foreach( EdgeSC* edge; edges ){
					writeln( "Compare [ " ~ edge.ends[0].id.to!string ~ ", " ~ edge.ends[1].id.to!string ~ " ]"  );					
					if( ((edge.ends[0].id == lastNode.id) && (edge.ends[1].id == node.id)) 
						|| 
						((edge.ends[1].id == lastNode.id) && (edge.ends[0].id == node.id)) ){
						foundNode = true;
						writeln( "EDGE MATCH" );
						break;
					}
				}
				// If there is not a connection, then add an edge
				if( !foundNode ){
					nuEdge = new EdgeSC( [node, lastNode] );
					edges ~= nuEdge; 
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
		// Draw all the tiles on the map
		foreach( LandSC* tile; hexes ){
			tile.draw();
		}
	}

	void draw_nodes(){
		// Draw all the nodes and structures at nodes
		foreach( NodeSC* node; nodes ){
			node.draw();
		}
	}

	void draw_edges(){
		// Draw all the edges and the roads at edges
		foreach( EdgeSC* edge; edges ){
			edge.draw();
		}
	}

	void draw_state(){
		draw_tiles();
		draw_nodes();
		draw_edges();
	}
}


void main(){

	///// Game Constants /////////////////////////
	
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
			200 + 9*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 2*EDGE_LEN*cos(pi/6.0), 
			200 + 9*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 4*EDGE_LEN*cos(pi/6.0), 
			200 + 9*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 - EDGE_LEN*cos(pi/6.0) + 6*EDGE_LEN*cos(pi/6.0), 
			200 + 9*EDGE_LEN*cos(pi/3.0) 
		),

		// Row 5
		Vector2( 
			400, 
			200 + 12*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 + 2*EDGE_LEN*cos(pi/6.0), 
			200 + 12*EDGE_LEN*cos(pi/3.0) 
		),
		Vector2( 
			400 + 4*EDGE_LEN*cos(pi/6.0), 
			200 + 12*EDGE_LEN*cos(pi/3.0) 
		),
	];

	REL_NODE_LOCS = [ // Locations of nodes relative to a land tile center
		Vector2( 0.0f, EDGE_LEN ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -1.0f*pi/3.0f ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -2.0*pi/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -3.0*pi/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -4.0*pi/3.0 ),
		Vector2Rotate( Vector2( 0.0f, EDGE_LEN ), -5.0*pi/3.0 ),
	];

	PLAYER_COLORS = [ // Colors for rendering player properties
		SC_Player.RED : Colors.RED   ,
		SC_Player.ORN : Colors.ORANGE,
		SC_Player.WHT : Colors.WHITE ,
		SC_Player.BLU : Colors.BLUE  ,
	];

	///// Board Setup ////////////////////////////

	BoardSC board = BoardSC(); 

	/// Lay Down Tiles ///

	board.add_tile( new LandSC(
		TILE_CENTERS[0], //- Location in 2D space
		10, // Dice rolls that yields the resource
		SC_Resource.ROCKS, // ---- Type of land this is
		Colors.GRAY // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[1], //- Location in 2D space
		2, // Dice rolls that yields the resource
		SC_Resource.SHEEP, // ---- Type of land this is
		Colors.GREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[2], //- Location in 2D space
		9, // Dice rolls that yields the resource
		SC_Resource.LUMBR, // ---- Type of land this is
		Colors.DARKGREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[3], //- Location in 2D space
		12, // Dice rolls that yields the resource
		SC_Resource.GRAIN, // ---- Type of land this is
		Colors.GOLD // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[4], //- Location in 2D space
		6, // Dice rolls that yields the resource
		SC_Resource.BRCKS, // ---- Type of land this is
		Colors.MAROON // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[5], //- Location in 2D space
		4, // Dice rolls that yields the resource
		SC_Resource.SHEEP, // ---- Type of land this is
		Colors.GREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[6], //- Location in 2D space
		10, // Dice rolls that yields the resource
		SC_Resource.BRCKS, // ---- Type of land this is
		Colors.MAROON // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[7], //- Location in 2D space
		9, // Dice rolls that yields the resource
		SC_Resource.GRAIN, // ---- Type of land this is
		Colors.GOLD // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[8], //- Location in 2D space
		11, // Dice rolls that yields the resource
		SC_Resource.LUMBR, // ---- Type of land this is
		Colors.DARKGREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[9], //- Location in 2D space
		7, // Dice rolls that yields the resource
		SC_Resource.WLDCD, // ---- Type of land this is
		Colors.BEIGE // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[10], //- Location in 2D space
		3, // Dice rolls that yields the resource
		SC_Resource.LUMBR, // ---- Type of land this is
		Colors.DARKGREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[11], //- Location in 2D space
		8, // Dice rolls that yields the resource
		SC_Resource.ROCKS, // ---- Type of land this is
		Colors.GRAY // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[12], //- Location in 2D space
		8, // Dice rolls that yields the resource
		SC_Resource.LUMBR, // ---- Type of land this is
		Colors.DARKGREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[13], //- Location in 2D space
		3, // Dice rolls that yields the resource
		SC_Resource.ROCKS, // ---- Type of land this is
		Colors.GRAY // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[14], //- Location in 2D space
		4, // Dice rolls that yields the resource
		SC_Resource.GRAIN, // ---- Type of land this is
		Colors.GOLD // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[15], //- Location in 2D space
		5, // Dice rolls that yields the resource
		SC_Resource.SHEEP, // ---- Type of land this is
		Colors.GREEN // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[16], //- Location in 2D space
		5, // Dice rolls that yields the resource
		SC_Resource.BRCKS, // ---- Type of land this is
		Colors.MAROON // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[17], //- Location in 2D space
		6, // Dice rolls that yields the resource
		SC_Resource.GRAIN, // ---- Type of land this is
		Colors.GOLD // --- Display color
	) );

	board.add_tile( new LandSC(
		TILE_CENTERS[18], //- Location in 2D space
		11, // Dice rolls that yields the resource
		SC_Resource.SHEEP, // ---- Type of land this is
		Colors.GREEN // --- Display color
	) );


	///// Rendering //////////////////////////////

	// call this before using raylib
	validateRaylibBinding();

	// Window / Display Params
    InitWindow( 1200, 900, "Settlers of C*t*n Simulation" );
    SetTargetFPS(60);

	writeln( "##### Settlers of C*t*n Simulation, Version 2022-12 #####" );
	writeln( "There are " ~ board.hexes.length.to!string ~ " tiles." );
	writeln( "There are " ~ board.nodes.length.to!string ~ " nodes." );
	writeln( "There are " ~ board.edges.length.to!string ~ " edges." );
	// writeln( board.nodes[0] == board.nodes[0] );


	while ( !WindowShouldClose() ){

		/// Rendering ///
		BeginDrawing();

		ClearBackground( Colors.DARKBLUE );

		// l1.draw();
		// l2.draw();
		// board.draw_tiles();
		// board.draw_nodes();
		// board.draw_edges();
		board.draw_state();

		EndDrawing();

	}

	CloseWindow();
}

import std.stdio; // ------ writeline
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;

import raylib; // --------- easy graphics

import toybox;

// Randomness
auto rnd = Random( unpredictableSeed );

////////// CUBIC TRACK /////////////////////////////////////////////////////////////////////////////
struct Node3{
	// A point on a `Grid3`
	int[3]   address;
	Vector3  point;
	bool     occupied;
	Edge3*[] edges;

	this( int[3] addr, Vector3 pnt ){
		address   = addr;
		point     = pnt;
		occupied  = false;
		edges     = [];
	}
}

struct Edge3{
	// A segment on a `Grid3`
	Node3* head;
	Node3  tail;
}

class Grid3{
	int /*------*/ halfLimit;
	Node3*[int[3]] nodes;
	Edge3*[] /*-*/ edges;

	// FIXME: SELF-POPULATE UPON INSTANTIATION
}

bool check_address( int[3] address, int halfLimit ){
	// Check that the address fits in the grid
	return (abs(address[0])<=halfLimit) && (abs(address[1])<=halfLimit) && (abs(address[2])<=halfLimit);
}

int[3] random_Grid3_address( int halfLimit ){
	return [
		uniform(-halfLimit, halfLimit, rnd),
		uniform(-halfLimit, halfLimit, rnd),
		uniform(-halfLimit, halfLimit, rnd)
	];
}

int[3] random_Grid3_neighbor( int[3] origin, int halfLimit ){
	// Choose a random neighbor that is within the bounds of the grid
	bool correct   = false;
	bool different = false;
	int[3] rtnAddr;
	int dir;

	// 0. Roll for neighbor
	while( (!correct) || (!different) ){
		// 1. Choose a direction to go
		dir = uniform( 0, 2, rnd);
		// 2. Go in that direction
		rtnAddr = [
			origin[0] + ((dir == 0) ? uniform(-1, 1, rnd) : 0),
			origin[1] + ((dir == 1) ? uniform(-1, 1, rnd) : 0),
			origin[2] + ((dir == 2) ? uniform(-1, 1, rnd) : 0)
		];
		correct   = check_address( rtnAddr, halfLimit );
		different = (rtnAddr != origin);
	}
	
	return rtnAddr;
}

int[3][] get_unoccupied_vonNeumann_neighbors( Grid3 grid, int[3] address ){
	// Return all the von Neumann neighbors to `grid` `address` not marked occupied, If none then return empty
	int[3]   chkAddr;
	int[3][] rtnAddr;

	// 1. Get all neighbors

	foreach( int dx; [-1,1] ){
		chkAddr = [ address[0]+dx, address[1], address[2] ];
		// 2. If neighbor is unoccupied, add it to the vector
		if( check_address( chkAddr, grid.halfLimit ) )  rtnAddr ~= chkAddr;
	}

	foreach( int dy; [-1,1] ){
		chkAddr = [ address[0], address[1]+dy, address[2] ];
		// 2. If neighbor is unoccupied, add it to the vector
		if( check_address( chkAddr, grid.halfLimit ) )  rtnAddr ~= chkAddr;
	}

	foreach( int dz; [-1,1] ){
		chkAddr = [ address[0], address[1], address[2]+dz ];
		// 2. If neighbor is unoccupied, add it to the vector
		if( check_address( chkAddr, grid.halfLimit ) )  rtnAddr ~= chkAddr;
	}

	// N. Return neighbors or empty
	return rtnAddr;
}

void gen_cycle_on_3D_grid( Grid3 grid, float epsilon ){
	// Generate a wandering cycle on a 3D regular grid
	// Decide whether to wander or return for this step

	// FIXME, START HERE: CHECK IF WE ARE VERY CLOSE TO CLOSING THE LOOP

	if( uniform( 0.0, 1.0, rnd) < epsilon ){
		// 1. FIXME: GET RANDOM NEIGHBOR AND CHECK THAT IT DOES NOT GET US STUCK
	}else{

	}
	

}

void main(){
	Grid3   grid    = new Grid3();
	int     halfLim = 10;
	int[3]  addAddr;
	Vector3 addPoint;
	float   edgeLen  = 1.0f;
	for( int x = -halfLim; x <= halfLim; x++ ){
		for( int y = -halfLim; y <= halfLim; y++ ){
			for( int z = -halfLim; z <= halfLim; z++ ){
				addAddr  = [x,y,z];
				addPoint = Vector3( x*edgeLen, y*edgeLen, z*edgeLen );
				grid.nodes[ addAddr ] = new Node3( addAddr, addPoint );
			}
		}	
	}

	writeln( grid.nodes[ [3,3,3] ].point );
}

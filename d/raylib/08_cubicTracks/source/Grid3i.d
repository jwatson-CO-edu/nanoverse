module Grid3i;

import std.stdio; // ------ writeline
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;
import core.stdc.stdlib; //abs

// import raylib; // --------- easy graphics

// import toybox;

// Randomness
Mt19937 rnd;

////////// CUBIC TRACK /////////////////////////////////////////////////////////////////////////////

////////// Grid3i HELPERS //////////////////////////////////////////////////////////////////////////

struct Node3i{
	// A point on a `Grid3i`
	int[3]   address;
	Vector3  point;
	bool     occupied;
	Edge3i*[] edges;

	this( int[3] addr, Vector3 pnt ){
		address   = addr;
		point     = pnt;
		occupied  = false;
		edges     = [];
	}
}

struct Edge3i{
	// A segment on a `Grid3i`
	Node3i* head;
	Node3i* tail;
}

int[3] random_Grid3i_address( int halfLimit ){
	// Get a random address within the `halfLimit`
	return [
		uniform(-halfLimit, halfLimit, rnd),
		uniform(-halfLimit, halfLimit, rnd),
		uniform(-halfLimit, halfLimit, rnd)
	];
}

bool check_address( int[3] address, int halfLimit ){
	// Check that the address fits in the grid
	return (abs(address[0])<=halfLimit) && (abs(address[1])<=halfLimit) && (abs(address[2])<=halfLimit);
}

class Grid3i{
	int /*------*/ halfLimit;
	float           edgeLength;
	Node3i*[int[3]] nodes;
	Edge3i*[] /*-*/ edges;

	this( int halfLim, float edgeLen ){
		halfLimit  = halfLim;
		edgeLength = edgeLen;
		int[3]  addAddr;
		Vector3 addPoint;
		for( int x = -halfLim; x <= halfLim; x++ ){
			for( int y = -halfLim; y <= halfLim; y++ ){
				for( int z = -halfLim; z <= halfLim; z++ ){
					addAddr  = [x,y,z];
					addPoint = Vector3( x*edgeLen, y*edgeLen, z*edgeLen );
					nodes[ addAddr ] = new Node3i( addAddr, addPoint );
				}
			}	
		}
	}

	bool add_edge( int[3] p1, int[3] p2  ){
		if( check_address( p1, halfLimit ) && check_address( p2, halfLimit ) ){
			edges ~= new Edge3i( nodes[ p1 ], nodes[ p2 ] );
			nodes[ p1 ].occupied = true;
			nodes[ p2 ].occupied = true;
		}else  return false;
 	}

	int[3] random_unoccupied_address(){
		// Return a random unoccpied address
		int[3] rtnAddr = random_Grid3i_address( halfLimit );
		while( nodes[ rtnAddr ].occupied ){
			rtnAddr = random_Grid3i_address( halfLimit );
		}
		return rtnAddr;
	}

	int[3][] get_vonNeumann_neighbor_addrs( int[3] address ){
		// Return all the von Neumann neighbors to `grid` `address`
		int[3]   chkAddr;
		int[3][] rtnAddr;

		// 1. Get all neighbors

		foreach( int dx; [-1,1] ){
			chkAddr = [ address[0]+dx, address[1], address[2] ];
			// 2. If neighbor is unoccupied, add it to the vector
			if( check_address( chkAddr, halfLimit ) && (!nodes[chkAddr].occupied) )  rtnAddr ~= chkAddr;
		}

		foreach( int dy; [-1,1] ){
			chkAddr = [ address[0], address[1]+dy, address[2] ];
			// 2. If neighbor is unoccupied, add it to the vector
			if( check_address( chkAddr, halfLimit ) && (!nodes[chkAddr].occupied) )  rtnAddr ~= chkAddr;
		}

		foreach( int dz; [-1,1] ){
			chkAddr = [ address[0], address[1], address[2]+dz ];
			// 2. If neighbor is unoccupied, add it to the vector
			if( check_address( chkAddr, halfLimit ) && (!nodes[chkAddr].occupied) )  rtnAddr ~= chkAddr;
		}

		// N. Return neighbors or empty
		return rtnAddr;
	}

	
	
}





int[3] random_Grid3i_neighbor( int[3] origin, int halfLimit ){
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



int manhattan_dist( int[3] p1, int[3] p2 ){
	// Return the manhattan distance between two integer 3D coordinates
	return abs( p1[0] - p2[0] ) + abs( p1[2] - p2[2] ) + abs( p1[2] - p2[2] );
}

void gen_cycle_on_3D_grid( Grid3i grid, float epsilon ){
	// Generate a wandering cycle on a 3D regular grid

	// Start at a random point on the grid
	int[3] start = grid.random_unoccupied_address();

	// Decide whether to wander or return for this step

	// FIXME, START HERE: CHECK IF WE ARE VERY CLOSE TO CLOSING THE LOOP

	if( uniform( 0.0, 1.0, rnd) < epsilon ){
		// 1. FIXME: GET RANDOM NEIGHBOR AND CHECK THAT IT DOES NOT GET US STUCK
	}else{

	}
	

}

// void main(){
// 	rnd = Random( unpredictableSeed );
// 	int     halfLim = 10;
// 	float   edgeLen  = 1.0f;
// 	Grid3i   grid    = new Grid3i( halfLim, edgeLen );
	
	
// 	Vector3 addPoint;
// 	int[3] a1 = grid.random_unoccupied_address();
// 	int[3] a2 = grid.get_vonNeumann_neighbor_addrs( a1 )[0];
// 	writeln( a1 );
// 	writeln( a2 );
// }

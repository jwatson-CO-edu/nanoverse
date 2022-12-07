import std.stdio; // ------ writeline
import core.time;
import core.math; // `sqrt`
import std.math.exponential; // `pow`
import std.random;

import raylib; // --------- easy graphics

import toybox;

////////// CUBIC TRACK /////////////////////////////////////////////////////////////////////////////
struct Node3{
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
	Node3* head;
	Node3  tail;
}

class Grid3{
	Node3*[int[3]] nodes;
	Edge3*[] /*-*/ edges;
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

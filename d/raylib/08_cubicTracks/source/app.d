struct Node3f{
	// A point on a `Grid3i`
	Vector3   point;
	bool /**/ occupied;
	Edge3f*[] edges;

	this( Vector3 pnt ){
		point     = pnt;
		occupied  = false;
		edges     = [];
	}
}

struct Edge3f{
	// A segment on a `Grid3i`
	Node3f* head;
	Node3f* tail;
}
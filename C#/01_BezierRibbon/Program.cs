using OpenTK.Mathematics;
using geo3d;
using ribbon;
using parametric;

Console.WriteLine( "Hello, World!" );

List<Tri> totMesh = [];

Ribbon R = new( twist_ : MathF.PI/2f ){
    spine = new Line.Segment( new Vector3(0, 0, 0), new Vector3(4, 0, 0) )
};
R.SetXdirAt0( new Vector3(0,1,0) );
R.BuildGeo( 1, 2 );
R.SetColor( new Vector4(1,0,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );


Ribbon G = new( twist_ : MathF.PI/2f ){
    spine = new Line.Segment( new Vector3(0, 0, 0), new Vector3(0, 4, 0) )
};
G.SetXdirAt0( new Vector3(0,0,1) );
G.BuildGeo( 1, 2 );
G.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( G.GetTotalMesh() );


Ribbon B = new( twist_ : MathF.PI/2f ){
    spine = new Line.Segment( new Vector3(0, 0, 0), new Vector3(0, 0, 4) )
};
B.SetXdirAt0( new Vector3(1,0,0) );
B.BuildGeo( 1, 2 );
B.SetColor( new Vector4(0,0,1,1), new Vector4(0,0,0,1) );
totMesh.AddRange( B.GetTotalMesh() );


TriMeshViewer.Show( totMesh );

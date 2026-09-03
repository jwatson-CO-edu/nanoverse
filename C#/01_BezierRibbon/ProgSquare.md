```C#

using OpenTK.Mathematics;
using geo3d;
using pose3d;
using ribbon;
using parametric;

Console.WriteLine( "Hello, World!" );

List<Tri> totMesh = [];
Random    rand    = new();


Ribbon R = new( twist_ : 0 ){
    spine = new Line.Segment( new Vector3(0, 0, 0), new Vector3(4, 0, 0) )
};
R.SetXdirAt0( new Vector3(0,1,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );


R = new( twist_ : 0 ){
    spine = new Line.Segment( new Vector3(4, 0, 0), new Vector3(4, 4, 0) )
};
R.SetXdirAt0( new Vector3(1,0,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );


R = new( twist_ : 0 ){
    spine = new Line.Segment( new Vector3(4, 4, 0), new Vector3(0, 4, 0) )
};
R.SetXdirAt0( new Vector3(0,1,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );


R = new( twist_ : 0 ){
    spine = new Line.Segment( new Vector3(0, 4, 0), new Vector3(0, 0, 0) )
};
R.SetXdirAt0( new Vector3(1,0,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );

Elements elem = new();

float capRad = 0.50f;


List<Tri> circ = elem.CircleCap( new Vector3(0, 0, 0), Vector3.UnitZ, capRad, new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( circ );

circ = elem.CircleCap( new Vector3(4, 0, 0), Vector3.UnitZ, capRad, new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( circ );

circ = elem.CircleCap( new Vector3(0, 4, 0), Vector3.UnitZ, capRad, new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( circ );

circ = elem.CircleCap( new Vector3(4, 4, 0), Vector3.UnitZ, capRad, new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( circ );


R = new( twist_ : 0 ){
    spine = new Bezier.Cubic( new Vector3(0,0,0), new Vector3(0,2,0), 
                              new Vector3(-2,2.5f,Constants._LAYER_SEP), new Vector3(-2,4.5f,Constants._LAYER_SEP) )
};
R.SetXdirAt0( new Vector3(1,0,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );



R = new( twist_ : 0 ){
    spine = new Bezier.Cubic( new Vector3(4,0,0), new Vector3(4,2,0), 
                              new Vector3(6,2.5f,Constants._LAYER_SEP), new Vector3(6,4.5f,Constants._LAYER_SEP) )
};
R.SetXdirAt0( new Vector3(1,0,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );


R = new( twist_ : 0 ){
    spine = new Bezier.Cubic( new Vector3(1,3.5f,0), new Vector3(1,1.5f,0), 
                              new Vector3(3,1.5f,Constants._LAYER_SEP), new Vector3(3,-0.5f,Constants._LAYER_SEP) )
};
R.SetXdirAt0( new Vector3(1,0,0) );
R.BuildGeo( 1 );
R.SetColor( new Vector4(0,1,0,1), new Vector4(0,0,0,1) );
totMesh.AddRange( R.GetTotalMesh() );


TriMeshViewer.Show( totMesh );


```
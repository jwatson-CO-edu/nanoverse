```C#

using OpenTK.Mathematics;
using geo3d;
using pose3d;
using ribbon;
using parametric;

Console.WriteLine( "Hello, World!" );

List<Tri> totMesh = [];
Random    rand    = new();

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


Ribbon C = new( twist_ : MathF.PI ){
    spine = new Ellipse.Circle( new Vector3(0,0,0), new Vector3(1,1,1), 4f )
};
C.SetXdirAt0( new Vector3(1,1,1) );
C.BuildGeo( 1.5f );
C.SetColor( new Vector4(0.75f,0.75f,0.75f,1), new Vector4(0,0,0,1) );
totMesh.AddRange( C.GetTotalMesh() );


Matrix4 matx_i;
Vector3 posn;
Ribbon L;
for( int i = 0; i < 3; ++i ){
    matx_i = C.GetFrameAt( rand.NextSingle() );
    posn   = MathMatx4.GetPosition( matx_i );
    L /**/ = new(){
        spine = new Line.Segment( posn, posn + MathMatx4.GetYBasis( matx_i ) * 2f )
    };
    L.SetXdirAt0( MathMatx4.GetXBasis( matx_i ) );
    L.BuildGeo( 1.5f );
    L.SetColor( new Vector4(0.75f,0.75f,0.75f,1), new Vector4(0,0,0,1) );
    totMesh.AddRange( L.GetTotalMesh() );
}


TriMeshViewer.Show( totMesh );

```
using geo3d;
using ideogram;
using OpenTK.Mathematics;


// See https://aka.ms/new-console-template for more information
Console.WriteLine("Hello, World!");

MeshGen mg = new();

List<Tri> totMesh = [];


///// Cuboid /////////////////////////////////////

List<Tri> cuboid = MeshGen.Cuboid( new Vector3(0,0,0), 1f, 2f, 3f );
cuboid = Tri.ColorMesh( cuboid, new Vector4(0.25f,0.25f,0.25f,1) );
totMesh.AddRange( cuboid );


///// Trapezoid Slab /////////////////////////////

List<Tri> slab = MeshGen.TrapezoidSlab( new Vector3(4,0,0), 0.5f,
                                         0.5f, 1f, 
                                         1.5f, 2f, 
                                         xPlaceTop    : -1, yPlaceTop    : -1,
                                         xPlaceBottom : 0, yPlaceBottom : 0 );
slab = Tri.ColorMesh( slab, new Vector4(1,0,0,1) );

totMesh.AddRange( slab );


///// Cylinder ///////////////////////////////////

List<Tri> cyl = mg.Cylinder( new Vector3(0,4,0), new Vector3(1,1,1), 0.5f, 2f, 16, 3 );
cyl = Tri.ColorMesh( cyl, new Vector4(0,1,0,1) );
totMesh.AddRange( cyl );


List<Tri> frust = mg.CircFrustum( new Vector3(4,4,0), new Vector3(-1,1,1), 
                                  0.5f, 1f, 2f, 
                                  16, 2 );
frust = Tri.ColorMesh( frust, new Vector4(1,1,0,1) );
totMesh.AddRange( frust );




TriMeshViewer.Show( totMesh );


Console.WriteLine( "Program COMPLETE!" );

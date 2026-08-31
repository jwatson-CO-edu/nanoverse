using geo3d;
using ideogram;
using OpenTK.Mathematics;


// See https://aka.ms/new-console-template for more information
Console.WriteLine( "Hello, World!" );

MeshGen mg = new();

List<Tri> totMesh = [];


///// Cuboid /////////////////////////////////////

List<Tri> cuboid = MeshGen.Cuboid( new Vector3(0,0,0), 1f, 2f, 3f );
cuboid = Tri.ColorMesh( cuboid, new Vector4(0.15f,0.15f,0.15f,1) );
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


///// Cicular Frustum ////////////////////////////

List<Tri> frust = mg.CircFrustum( new Vector3(4,4,0), new Vector3(-1,1,1), 
                                  0.5f, 1f, 2f, 
                                  16, 2 );
frust = Tri.ColorMesh( frust, new Vector4(1,1,0,1) );
totMesh.AddRange( frust );


///// Icosahedron ////////////////////////////////

List<Tri> icos = MeshGen.Icosahedron( 1f, new Vector3(0,0,4) );
icos = Tri.ColorMesh( icos, new Vector4(0,0,1,1) );
totMesh.AddRange( icos );


///// Sphere /////////////////////////////////////

List<Tri> sphr = MeshGen.Sphere( 1f, new Vector3(4,0,4) );
sphr = Tri.ColorMesh( sphr, new Vector4(1,0,1,1) );
totMesh.AddRange( sphr );


///// Torus //////////////////////////////////////

List<Tri> torus = MeshGen.EllipticalTorusXY( new Vector3(0,4,4), 2, 1, 0.5f );
torus = Tri.ColorMesh( torus, new Vector4(0,1,1,1) );
totMesh.AddRange( torus );


///// Plane //////////////////////////////////////

List<Tri> plane = MeshGen.PlaneXY( new Vector3( 4, 4, -2  ), 14f, 14f, 0.5f );
plane = Tri.ColorMesh( plane, new Vector4(0.5f,0.5f,0.5f,1) );
totMesh.AddRange( plane );


///// Wedge //////////////////////////////////////

List<Tri> wedge = MeshGen.Wedge( new Vector3(4,4,4), new Vector3(0,0,1), new Vector3(1,0,0), 
                                 MathF.PI/2, 1, 2 );
wedge = Tri.ColorMesh( wedge, new Vector4(0.85f,0.85f,0.85f,1) );
totMesh.AddRange( wedge );


///// Twisted Frustum ////////////////////////////

List<Tri> tFrust = MeshGen.TwistFrustum( new Vector3(8,4,0), new Vector3(1,2,3), Vector3.UnitX, MathF.PI/4f,
                                         1f, 2f, 2f, 
                                         6, 16 );
tFrust = Tri.ColorMesh( tFrust, new Vector4(1,0.5f,0,1) );
totMesh.AddRange( tFrust );


///// Display Aggregate Mesh /////////////////////

TriMeshViewer.Show( totMesh );


Console.WriteLine( "Program COMPLETE!" );

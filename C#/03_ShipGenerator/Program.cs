using geo3d;
using ideogram;
using OpenTK.Mathematics;


// See https://aka.ms/new-console-template for more information
Console.WriteLine("Hello, World!");

MeshGen mg = new();

List<Tri> cuboid = MeshGen.Cuboid( new Vector3(0,0,0), 1f, 2f, 3f );
cuboid = Tri.ColorMesh( cuboid, new Vector4(1,0,0,1) );

TriMeshViewer.Show( cuboid );


Console.WriteLine( "Program COMPLETE!" );

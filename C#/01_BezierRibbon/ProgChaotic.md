```C#

using geo3d;
using ribbon;

Console.WriteLine( "Hello, World!" );

GlyphGen gg = new( NgridHalf : 3 );
List<Tri> totMesh = gg.MakeGlyph();
Console.WriteLine( $"{totMesh.Count} triangles to draw!" );


TriMeshViewer.Show( totMesh );


```
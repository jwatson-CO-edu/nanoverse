using OpenTK.Mathematics;

using paraboloid;
using pso;
using geo3d;
using geo2d;
using cut_svg;

/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////

[Y] Run a dish solution (Test PSO) 
[Y] Construct Discretized Dish
[Y] Design ribs
[Y] Display dish + Rib(s) w Phong Shading
[Y] Allow user to rotate dish
[>] Lay out discretized units onto standard size rectangles in OpenGL
    [Y] Choose cutter
    [>] Confirm working area
    [ ] Choose "Curroplast" sheet (size(s))
    [ ] Confirm working area of sheet
[Y] Confirm SVG instructions for laser cutting
    [Y] Watch library instruction video
[ ] Lay out discretized units onto standard size rectangles in SVG
[ ] Lay out ribs onto standard size rectangles in SVG
[ ] Construct Dish, Install mic at focus
[ ] Test Dish + Mic
    [ ] Measure Gain ???

*/


////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

// See https://aka.ms/new-console-template for more information
Console.WriteLine( "Hello, World!" );

DishCalculator dc = new(
    Constants._BIRD_FREQ_LO, 
    100, 
    10f/180 * MathF.PI, 
    1.0f, 
    0.333f, 
    0.5f
); 

DctVecF soln = dc.DesignParabolicReflector();

List<Quad> qDish    = dc.SegmentDesignedReflector( soln );
// dc.PetalAsSVG( qDish );
List<Quad> supports = dc.DesignReflectorSupports( 0.00125f, qDish );
qDish.AddRange( supports );
List<Tri> tMesh = Quad.AsTriMesh( qDish );

tMesh.AddRange( dc.backPlate );

tMesh = Tri.ColorMesh( tMesh, new Vector4(0,0,1,1) );


CutSVG outSVG = new();
List<Segment> petal = dc.PetalSegments( qDish );
Vector2[]     aabb  = Segment.BBox( petal );
Vector2 /*-*/ half  = new( aabb[1][0] - aabb[0][0] + 0.5f * CutSVG._IN_TO_M, aabb[1][1] - aabb[0][1] + 0.5f * CutSVG._IN_TO_M );
half /= 2f;
Console.WriteLine( half );

outSVG.PatternGroup( petal, half, new Vector2( half[0] + 0.5f * CutSVG._IN_TO_M, 0f ), MathF.PI, 3 );

// outSVG.AddSegments_m( Segment.ShiftSegments( , new Vector2( 1f/CutSVG._M_TO_IN * 8.5f/2f, 1f/CutSVG._M_TO_IN * 11f/2f ) ) );
outSVG.WriteSVG();

CutSVG strutSVG = new();
List<Segment> strut = dc.SupportSegments( supports );

aabb  = Segment.BBox( strut );
half  = new( aabb[1][1] - aabb[0][1] + 0.5f * CutSVG._IN_TO_M, aabb[1][0] - aabb[0][0] + 1f * CutSVG._IN_TO_M );
half /= 2f;

strut = Segment.RotateSegments( strut, new Vector2(), MathF.PI/2f );

strutSVG.PatternGroup( strut, half, new Vector2( half[0] + 0.5f * CutSVG._IN_TO_M, 0f ), 0f, 4 );

strutSVG.WriteSVG( "strut.svg" );

CutSVG backSVG = new();
List<Segment> back = dc.BackSegments();

// backSVG.AddSegments_m( back );
backSVG.PatternGroup( back, new Vector2( 2f * CutSVG._IN_TO_M, 2f * CutSVG._IN_TO_M ), new Vector2( 0f, 3.25f * CutSVG._IN_TO_M ), 0f, 3 );

backSVG.WriteSVG( "back.svg" );

TriMeshViewer.Show( tMesh );

Console.WriteLine( "\n\nTASK COMPLETE!\n" );


using paraboloid;
using pso;
using geo3d;
using OpenTK.Mathematics;

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
List<Quad> supports = dc.DesignReflectorSupports( 0.004f, qDish );
qDish.AddRange( supports );
List<Tri> tMesh = Quad.AsTriMesh( qDish );

tMesh.AddRange( dc.backPlate );

tMesh = Tri.ColorMesh( tMesh, new Vector4(0,0,1,1) );


TriMeshViewer.Show( tMesh );

Console.WriteLine( "\n\nTASK COMPLETE!\n" );


using paraboloid;
using pso;
/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////

[Y] Run a dish solution (Test PSO) 
[Y] Construct Discretized Dish
[Y] Design ribs
[>] Display dish + Rib(s) w Phong Shading
[ ] Allow user to rotate dish
[ ] Lay out discretized units onto standard size rectangles in OpenGL
    [ ] Choose cutter
    [ ] Confirm working area
    [ ] Choose "Curroplast" sheet (size(s))
    [ ] Confirm working area of sheet
[ ] Confirm SVG instructions for laser cutting
    [ ] Watch library instruction video
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

dc.SegmentDesignedReflector( soln );

Console.WriteLine( "\n\nTASK COMPLETE!\n" );


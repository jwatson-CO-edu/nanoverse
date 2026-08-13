using paraboloid;
/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////

[ ] Run a dish solution (Test PSO) 
[ ] Construct Discretized Dish
[ ] Display dish w Phong Shading
[ ] Allow user to rotate dish
[ ] Lay out discretized units onto standard size rectangles in OpenGL
[ ] Confirm SVG instructions for laser cutting
[ ] Lay out discretized units onto standard size rectangles in SVG
[ ] Design ribs
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

dc.DesignParabolicReflector();

Console.WriteLine( "\n\nTASK COMPLETE!\n" );


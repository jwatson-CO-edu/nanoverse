using sigil;

// See https://aka.ms/new-console-template for more information
Console.WriteLine( "Hello, World!" );

Pictogram sgl = new();

while( sgl.strokes.Count == 0 ){
    sgl.Generate();
    sgl.FilterSmall( 0.25f );
    // sgl.FilterFar( 1.0f );
    sgl.FilterFar( 0.75f );
}




// sgl.ShiftToCentroid();
sgl.ShiftToCenter();

SigilWindow rndr = new( sgl.GetAllTriangles(), "test.jpg" );
rndr.Run();

Console.WriteLine( "TASK COMPLETE!\n" );

using sigil;

// See https://aka.ms/new-console-template for more information
Console.WriteLine( "Hello, World!" );

Pictogram sgl = new();
sgl.Generate();

SigilWindow rndr = new( sgl.GetAllTriangles(), "test.jpg" );
rndr.Run();

Console.WriteLine( "TASK COMPLETE!\n" );

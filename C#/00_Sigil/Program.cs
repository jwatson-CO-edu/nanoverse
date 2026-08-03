using sigil;

// See https://aka.ms/new-console-template for more information
Console.WriteLine( "Hello, World!" );

Pictogram sgl = new();
sgl.Generate();

Renderer rndr = new();
rndr.GetSquareBuffers( 1024 );

Console.WriteLine( "TASK COMPLETE!\n" );

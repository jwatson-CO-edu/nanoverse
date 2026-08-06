using sigil;

// See https://aka.ms/new-console-template for more information
Console.WriteLine( "Hello, World!" );

string dir = "";

if( args.Length > 0 ){
    Console.WriteLine( $"First argument: {args[0]}" );
    dir = args[0];
}else{
    Console.WriteLine( "No arguments passed." );
}

Pictogram sgl = new();

while( sgl.strokes.Count == 0 ){
    sgl.Generate();
    sgl.FilterSmall( 0.25f );
    sgl.FilterFar( 0.75f );
}




// sgl.ShiftToCentroid();
sgl.ShiftToCenter();
string fileName;
if( dir.Length > 0 ){
    fileName = $"{dir}/sigil_{Guid.NewGuid():N}.jpg";
}else{
    fileName = $"sigil_{Guid.NewGuid():N}.jpg";
}

SigilWindow rndr = new( sgl.GetAllTriangles(), fileName );
rndr.Run();

Console.WriteLine( $"Created: {fileName}\n" );

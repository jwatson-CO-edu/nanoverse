namespace SigilGenerator;

public static class Program
{
    public static void Main(string[] args)
    {
        int seed = (args.Length > 0 && int.TryParse(args[0], out int parsed))
            ? parsed
            : Environment.TickCount;

        var rng = new Random(seed);

        var strokes = SigilFactory.Generate(rng);
        var triangles = MeshBuilder.BuildTriangles(strokes);

        string fileName = $"sigil_{seed}_{Guid.NewGuid():N}.jpg";
        string outputPath = Path.Combine(Environment.CurrentDirectory, fileName);

        using var window = new SigilWindow(triangles, outputPath);
        window.Run();

        Console.WriteLine($"Sigil written to: {outputPath}  (seed={seed})");
    }
}

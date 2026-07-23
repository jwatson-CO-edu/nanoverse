using OpenTK.Mathematics;

namespace SigilGenerator;

/// <summary>One vertex along a stroke's centerline, in normalized [0,1] canvas space.</summary>
public class StrokePoint
{
    public Vector2 Pos;
    public float Thickness; // full ribbon width at this point, normalized units
}

/// <summary>
/// A single drawn "pen path" - a polyline (straight segments; curves are approximated by
/// many short segments so all downstream math only ever deals with line segments).
/// Layer determines paint order: a lower Layer is painted first and is "under" anything
/// painted later that crosses it, so it is the one that gets a gap cut into it.
/// </summary>
public class Stroke
{
    public List<StrokePoint> Points = new();
    public int Layer;
    public float Z; // small z-offset, purely for a subtle pseudo-3D layering look
    public float GapHalfWidth = 0.010f;

    // segment index -> list of distances-along-that-segment where a break should be cut
    public Dictionary<int, List<float>> SegmentGapDistances = new();
}

public static class SigilFactory
{
    const float Margin = 0.13f;
    const float MinB = Margin;
    const float MaxB = 1f - Margin;

    public static List<Stroke> Generate(Random rng)
    {
        var strokes = new List<Stroke>();
        var branchNodes = new List<Vector2>();

        Vector2 center = new(0.5f, 0.5f);
        branchNodes.Add(center);

        int strokeCount = rng.Next(4, 8);

        for (int s = 0; s < strokeCount; s++)
        {
            Vector2 start = (branchNodes.Count > 1 && rng.NextDouble() < 0.55)
                ? branchNodes[rng.Next(branchNodes.Count)]
                : center + RandomOffset(rng, 0.07f);

            var stroke = new Stroke
            {
                Layer = s,
                Z = (s - strokeCount / 2f) * 0.0015f
            };

            float baseThickness = MathUtil.Lerp(0.006f, 0.020f, (float)rng.NextDouble());
            float angle = (float)(rng.NextDouble() * MathF.PI * 2);
            Vector2 pos = start;

            // Tapered start point (organic pen lift-off look), unless this is a thick trunk.
            bool taperStart = rng.NextDouble() < 0.5;
            stroke.Points.Add(new StrokePoint { Pos = pos, Thickness = taperStart ? 0f : baseThickness });

            int segCount = rng.Next(2, 6);

            for (int i = 0; i < segCount; i++)
            {
                bool sharpTurn = rng.NextDouble() < 0.5;
                if (sharpTurn)
                {
                    // sudden angle change, ~20-110 degrees, either direction
                    float delta = (float)(rng.NextDouble() * 1.55 + 0.35);
                    angle += rng.Next(2) == 0 ? delta : -delta;

                    float len = MathUtil.Lerp(0.05f, 0.16f, (float)rng.NextDouble());
                    Vector2 dir = new(MathF.Cos(angle), MathF.Sin(angle));
                    pos = MathUtil.ClampToBounds(pos + dir * len, MinB, MaxB);
                    float thickness = baseThickness * MathUtil.Lerp(0.7f, 1.15f, (float)rng.NextDouble());
                    stroke.Points.Add(new StrokePoint { Pos = pos, Thickness = thickness });
                }
                else
                {
                    // gentle curve: several small steps, each with a small angle nudge
                    int subSteps = rng.Next(3, 6);
                    float len = MathUtil.Lerp(0.06f, 0.16f, (float)rng.NextDouble());
                    for (int sub = 0; sub < subSteps; sub++)
                    {
                        angle += (float)((rng.NextDouble() - 0.5) * 0.35);
                        Vector2 dir = new(MathF.Cos(angle), MathF.Sin(angle));
                        pos = MathUtil.ClampToBounds(pos + dir * (len / subSteps), MinB, MaxB);
                        float thickness = baseThickness * MathUtil.Lerp(0.75f, 1.1f, (float)rng.NextDouble());
                        stroke.Points.Add(new StrokePoint { Pos = pos, Thickness = thickness });
                    }
                }

                if (rng.NextDouble() < 0.18)
                    branchNodes.Add(pos); // future strokes may branch from this point
            }

            if (rng.NextDouble() < 0.5 && stroke.Points.Count > 1)
                stroke.Points[^1].Thickness = 0f; // taper the tip

            strokes.Add(stroke);
            branchNodes.Add(pos);

            // Occasionally cap the stroke with a small circle you could branch from later.
            if (rng.NextDouble() < 0.32)
            {
                Vector2 dir = new(MathF.Cos(angle), MathF.Sin(angle));
                Vector2 circleCenter = MathUtil.ClampToBounds(
                    pos + dir * MathUtil.Lerp(0.02f, 0.045f, (float)rng.NextDouble()), MinB, MaxB);
                float radius = MathUtil.Lerp(0.018f, 0.045f, (float)rng.NextDouble());

                strokes.Add(BuildCircleStroke(circleCenter, radius, baseThickness * 0.8f, s, stroke.Z + 0.0002f, rng));

                // remember a few perimeter points as branch nodes for later strokes
                for (int k = 0; k < 3; k++)
                {
                    float a = (float)(rng.NextDouble() * MathF.PI * 2);
                    branchNodes.Add(circleCenter + new Vector2(MathF.Cos(a), MathF.Sin(a)) * radius);
                }
            }
        }

        ComputeCrossingGaps(strokes);
        return strokes;
    }

    static Stroke BuildCircleStroke(Vector2 center, float radius, float thickness, int layer, float z, Random rng)
    {
        var circle = new Stroke { Layer = layer, Z = z, GapHalfWidth = 0.008f };
        const int segments = 28;
        float startAngle = (float)(rng.NextDouble() * MathF.PI * 2);
        for (int i = 0; i <= segments; i++)
        {
            float a = startAngle + i / (float)segments * MathF.PI * 2;
            Vector2 p = center + new Vector2(MathF.Cos(a), MathF.Sin(a)) * radius;
            circle.Points.Add(new StrokePoint { Pos = p, Thickness = thickness });
        }
        return circle;
    }

    static Vector2 RandomOffset(Random rng, float r) =>
        new((float)(rng.NextDouble() * 2 - 1) * r, (float)(rng.NextDouble() * 2 - 1) * r);

    /// <summary>
    /// Finds every place two different strokes cross away from a shared node, and records
    /// a gap on whichever stroke has the lower Layer (drawn first / passes "under").
    /// </summary>
    static void ComputeCrossingGaps(List<Stroke> strokes)
    {
        for (int i = 0; i < strokes.Count; i++)
        {
            for (int j = i + 1; j < strokes.Count; j++)
            {
                Stroke A = strokes[i], B = strokes[j];
                if (A.Layer == B.Layer) continue; // same pen stroke's own circle etc - don't cut

                for (int ai = 0; ai < A.Points.Count - 1; ai++)
                {
                    for (int bi = 0; bi < B.Points.Count - 1; bi++)
                    {
                        if (!MathUtil.SegmentIntersect(
                                A.Points[ai].Pos, A.Points[ai + 1].Pos,
                                B.Points[bi].Pos, B.Points[bi + 1].Pos,
                                out float t, out float u, out _))
                            continue;

                        bool aIsUnder = A.Layer < B.Layer;
                        Stroke under = aIsUnder ? A : B;
                        int segIdx = aIsUnder ? ai : bi;
                        float tt = aIsUnder ? t : u;

                        float segLen = Vector2.Distance(
                            under.Points[segIdx].Pos, under.Points[segIdx + 1].Pos);

                        if (!under.SegmentGapDistances.TryGetValue(segIdx, out var list))
                        {
                            list = new List<float>();
                            under.SegmentGapDistances[segIdx] = list;
                        }
                        list.Add(tt * segLen);
                    }
                }
            }
        }
    }
}

using OpenTK.Mathematics;

namespace SigilGenerator;

/// <summary>Small 2D geometry helpers used by generation and mesh building.</summary>
public static class MathUtil
{
    public static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    public static Vector2 Perp(Vector2 v) => new(-v.Y, v.X);

    /// <summary>
    /// Segment/segment intersection test. Returns false for parallel segments and for
    /// intersections that fall essentially at a shared endpoint (t or u near 0 or 1) -
    /// those are treated as deliberate "joins" in a node graph, not crossings, and should
    /// never get a break drawn in them.
    /// </summary>
    public static bool SegmentIntersect(
        Vector2 a0, Vector2 a1, Vector2 b0, Vector2 b1,
        out float t, out float u, out Vector2 point)
    {
        t = 0f; u = 0f; point = Vector2.Zero;

        Vector2 r = a1 - a0;
        Vector2 s = b1 - b0;
        float rxs = Cross(r, s);
        if (MathF.Abs(rxs) < 1e-9f) return false; // parallel / collinear

        Vector2 qp = b0 - a0;
        t = Cross(qp, s) / rxs;
        u = Cross(qp, r) / rxs;

        const float eps = 0.02f; // margin around endpoints treated as "same node"
        if (t <= eps || t >= 1f - eps || u <= eps || u >= 1f - eps) return false;

        point = a0 + t * r;
        return true;
    }

    public static float Lerp(float a, float b, float t) => a + (b - a) * t;

    public static Vector2 ClampToBounds(Vector2 p, float lo, float hi) =>
        new(Math.Clamp(p.X, lo, hi), Math.Clamp(p.Y, lo, hi));
}

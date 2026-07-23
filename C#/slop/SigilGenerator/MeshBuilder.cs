using OpenTK.Mathematics;

namespace SigilGenerator;

public static class MeshBuilder
{
    public static List<Vector3> BuildTriangles(List<Stroke> strokes)
    {
        var tris = new List<Vector3>();
        foreach (var stroke in strokes)
            BuildStroke(stroke, tris);
        return tris;
    }

    static void BuildStroke(Stroke stroke, List<Vector3> tris)
    {
        var pts = stroke.Points;
        if (pts.Count < 2) return;
        float z = stroke.Z;

        for (int i = 0; i < pts.Count - 1; i++)
        {
            Vector2 p0 = pts[i].Pos, p1 = pts[i + 1].Pos;
            float t0 = pts[i].Thickness, t1 = pts[i + 1].Thickness;
            float segLen = Vector2.Distance(p0, p1);
            if (segLen < 1e-6f) continue;

            List<float> gapCenters = stroke.SegmentGapDistances.TryGetValue(i, out var g)
                ? g.OrderBy(x => x).ToList()
                : new List<float>();

            float half = stroke.GapHalfWidth;
            var intervals = new List<(float s, float e)>();
            float cursor = 0f;
            foreach (float gc in gapCenters)
            {
                float gs = Math.Clamp(gc - half, 0f, segLen);
                float ge = Math.Clamp(gc + half, 0f, segLen);
                if (gs > cursor) intervals.Add((cursor, gs));
                cursor = Math.Max(cursor, ge);
            }
            if (cursor < segLen) intervals.Add((cursor, segLen));

            foreach (var (s, e) in intervals)
            {
                if (e - s < 1e-5f) continue;
                float fs = s / segLen, fe = e / segLen;
                Vector2 ps = Vector2.Lerp(p0, p1, fs);
                Vector2 pe = Vector2.Lerp(p0, p1, fe);
                float ws = MathUtil.Lerp(t0, t1, fs);
                float we = MathUtil.Lerp(t0, t1, fe);

                AddQuad(tris, ps, pe, ws, we, z);

                // Round off cut ends created by a break so it reads as a deliberate gap,
                // but don't cap the true start/end of the stroke here (handled below,
                // and may legitimately taper to a point).
                bool isTrueStart = i == 0 && s == 0f;
                bool isTrueEnd = i == pts.Count - 2 && e >= segLen - 1e-4f;
                if (!isTrueStart) AddCap(tris, ps, ws / 2f, z);
                if (!isTrueEnd) AddCap(tris, pe, we / 2f, z);
            }
        }

        // Round interior joints so sharp-angle corners don't show a notch.
        for (int i = 1; i < pts.Count - 1; i++)
            AddCap(tris, pts[i].Pos, pts[i].Thickness / 2f, z);

        // Round the true tips (no-op if tapered to zero thickness).
        AddCap(tris, pts[0].Pos, pts[0].Thickness / 2f, z);
        AddCap(tris, pts[^1].Pos, pts[^1].Thickness / 2f, z);
    }

    static void AddQuad(List<Vector3> tris, Vector2 pa, Vector2 pb, float wa, float wb, float z)
    {
        Vector2 dir = pb - pa;
        if (dir.LengthSquared < 1e-12f) return;
        Vector2 n = Vector2.Normalize(MathUtil.Perp(dir));

        Vector2 a0 = pa + n * (wa / 2f);
        Vector2 a1 = pa - n * (wa / 2f);
        Vector2 b0 = pb + n * (wb / 2f);
        Vector2 b1 = pb - n * (wb / 2f);

        tris.Add(new Vector3(a0.X, a0.Y, z));
        tris.Add(new Vector3(a1.X, a1.Y, z));
        tris.Add(new Vector3(b0.X, b0.Y, z));

        tris.Add(new Vector3(a1.X, a1.Y, z));
        tris.Add(new Vector3(b1.X, b1.Y, z));
        tris.Add(new Vector3(b0.X, b0.Y, z));
    }

    static void AddCap(List<Vector3> tris, Vector2 center, float radius, float z, int segs = 10)
    {
        if (radius <= 0.0002f) return;
        for (int k = 0; k < segs; k++)
        {
            float a0 = k / (float)segs * MathF.PI * 2f;
            float a1 = (k + 1) / (float)segs * MathF.PI * 2f;
            Vector2 p0 = center + new Vector2(MathF.Cos(a0), MathF.Sin(a0)) * radius;
            Vector2 p1 = center + new Vector2(MathF.Cos(a1), MathF.Sin(a1)) * radius;

            tris.Add(new Vector3(center.X, center.Y, z));
            tris.Add(new Vector3(p0.X, p0.Y, z));
            tris.Add(new Vector3(p1.X, p1.Y, z));
        }
    }
}

using geo3d;
using OpenTK.Mathematics;

namespace ideogram {

/// <summary>
/// Generate meshes suitable for composable models
/// </summary>
public class MeshGen {

    public Random rand = new(); // Local RNG


    /// <summary>
    /// Return a cuboid mesh with arbitrary, axis-aligned side lengths
    /// </summary>
    public static List<Tri> Cuboid( Vector3 center, float Xside = 1f, float Yside = 1f, float Zside = 1f ){
        List<Tri> rtnShp = [];
        float     xHalf  = Xside / 2f;
        float     yHalf  = Yside / 2f;
        float     zHalf  = Zside / 2f;
        rtnShp.Capacity = 12;
        Vector3 p0 = center - Vector3.UnitX * xHalf - Vector3.UnitY * yHalf - Vector3.UnitZ * zHalf;
        Vector3 p1 = center - Vector3.UnitX * xHalf + Vector3.UnitY * yHalf - Vector3.UnitZ * zHalf;
        Vector3 p2 = center + Vector3.UnitX * xHalf + Vector3.UnitY * yHalf - Vector3.UnitZ * zHalf;
        Vector3 p3 = center + Vector3.UnitX * xHalf - Vector3.UnitY * yHalf - Vector3.UnitZ * zHalf;
        Vector3 p4 = center - Vector3.UnitX * xHalf - Vector3.UnitY * yHalf + Vector3.UnitZ * zHalf;
        Vector3 p5 = center + Vector3.UnitX * xHalf - Vector3.UnitY * yHalf + Vector3.UnitZ * zHalf;
        Vector3 p6 = center + Vector3.UnitX * xHalf + Vector3.UnitY * yHalf + Vector3.UnitZ * zHalf;
        Vector3 p7 = center - Vector3.UnitX * xHalf + Vector3.UnitY * yHalf + Vector3.UnitZ * zHalf;
        
        /*   6 ---- 5
           / |    / |
         7 ---- 4   |
         |   2 -|-- 3
         | /    | / 
         1 ---- 0   */
        
        // Bottom //
        rtnShp.Add( new Tri( p0, p1, p2 ) );
        rtnShp.Add( new Tri( p2, p3, p0 ) );

        // Top //
        rtnShp.Add( new Tri( p4, p5, p6 ) );
        rtnShp.Add( new Tri( p6, p7, p4 ) );

        // Front //
        rtnShp.Add( new Tri( p0, p4, p7 ) );
        rtnShp.Add( new Tri( p7, p1, p0 ) );

        // Back //
        rtnShp.Add( new Tri( p2, p6, p5 ) );
        rtnShp.Add( new Tri( p5, p3, p2 ) );

        // Right //
        rtnShp.Add( new Tri( p4, p0, p3 ) );
        rtnShp.Add( new Tri( p3, p5, p4 ) );
        
        // Left //
        rtnShp.Add( new Tri( p1, p7, p6 ) );
        rtnShp.Add( new Tri( p6, p2, p1 ) );

        return rtnShp;
    }


    /// <summary>
    /// Return a trapezoidal slab mesh with specified XY alignment of the top and bottom
    /// </summary>
    public static List<Tri> TrapezoidSlab( Vector3 center, float thickness = 1f,
                                           float xSideTop    = 1f, float ySideTop    = 1f, 
                                           float xSideBottom = 1f, float ySideBottom = 1f, 
                                           int xPlaceTop    = 0, int yPlaceTop    = 0,
                                           int xPlaceBottom = 0, int yPlaceBottom = 0 ){
        List<Tri> rtnShp = [];
        rtnShp.Capacity = 12;
        
        float xHalfTop  = xSideTop / 2f;
        float yHalfTop  = ySideTop / 2f;
        float xHalfBtm  = xSideBottom / 2f;
        float yHalfBtm  = ySideBottom / 2f;
        float xShiftTop = Math.Max( xHalfBtm - xHalfTop, 0f ); // Calculate shift for the smaller X
        float yShiftTop = Math.Max( yHalfBtm - yHalfTop, 0f ); // Calculate shift for the smaller Y
        float xShiftBtm = Math.Max( xHalfTop - xHalfBtm, 0f ); // Calculate shift for the smaller X
        float yShiftBtm = Math.Max( yHalfTop - yHalfBtm, 0f ); // Calculate shift for the smaller Y
        float zHalf     = thickness / 2f;

        Vector3 top = center + Vector3.UnitX * xShiftTop * xPlaceTop    + Vector3.UnitY * yShiftTop * yPlaceTop    + Vector3.UnitZ * zHalf;
        Vector3 btm = center + Vector3.UnitX * xShiftBtm * xPlaceBottom + Vector3.UnitY * yShiftBtm * yPlaceBottom - Vector3.UnitZ * zHalf;

        Vector3 p0 = top - Vector3.UnitX * xHalfTop - Vector3.UnitY * yHalfTop;
        Vector3 p1 = top - Vector3.UnitX * xHalfTop + Vector3.UnitY * yHalfTop;
        Vector3 p2 = top + Vector3.UnitX * xHalfTop + Vector3.UnitY * yHalfTop;
        Vector3 p3 = top + Vector3.UnitX * xHalfTop - Vector3.UnitY * yHalfTop;

        Vector3 p4 = btm - Vector3.UnitX * xHalfBtm - Vector3.UnitY * yHalfBtm;
        Vector3 p5 = btm + Vector3.UnitX * xHalfBtm - Vector3.UnitY * yHalfBtm;
        Vector3 p6 = btm + Vector3.UnitX * xHalfBtm + Vector3.UnitY * yHalfBtm;
        Vector3 p7 = btm - Vector3.UnitX * xHalfBtm + Vector3.UnitY * yHalfBtm;
        
        /*   6 ----- 5
           /  \    /   \
         7 ----- 4       \
         |     2 -\------- 3
         |   /     \     /
         | /        \  /
         1 --------- 0   */
        
        // Bottom //
        rtnShp.Add( new Tri( p0, p1, p2 ) );
        rtnShp.Add( new Tri( p2, p3, p0 ) );

        // Top //
        rtnShp.Add( new Tri( p4, p5, p6 ) );
        rtnShp.Add( new Tri( p6, p7, p4 ) );

        // Front //
        rtnShp.Add( new Tri( p0, p4, p7 ) );
        rtnShp.Add( new Tri( p7, p1, p0 ) );

        // Back //
        rtnShp.Add( new Tri( p2, p6, p5 ) );
        rtnShp.Add( new Tri( p5, p3, p2 ) );

        // Right //
        rtnShp.Add( new Tri( p4, p0, p3 ) );
        rtnShp.Add( new Tri( p3, p5, p4 ) );
        
        // Left //
        rtnShp.Add( new Tri( p1, p7, p6 ) );
        rtnShp.Add( new Tri( p6, p2, p1 ) );

        return rtnShp;
    } 


    /// <summary>
    /// Return a cylindrical mesh with a specified longitudinal axis
    /// </summary>
    public List<Tri> Cylinder( Vector3 center, Vector3 axis, float radius = 1f, float height = 1f, int arcDiv = 16, int hgtDiv = 2 ){
        List<Tri>     rtnShp = [];
        List<Vector3> segCtr = [];
        rtnShp.Capacity = 2 * arcDiv + 2 * arcDiv * hgtDiv; // 2x Endcaps + `arcDiv` x `hgtDiv` Quads
        segCtr.Capacity = hgtDiv + 1;
        axis.Normalize();
        Vector3 top = center + axis * (height * 0.5f);
        Vector3 btm = center - axis * (height * 0.5f);
        segCtr.Add( btm );
        Vector3 zSpr = Vector3.Cross( axis, MathVec3.NoiseXYZ( rand ) ).Normalized() * radius;
        Vector3 spar, p0, p1, p2, p3;
        Vector3 lSpr = zSpr;
        float   dTheta = 2f * MathF.PI / arcDiv;
        float   dHgt   = height / hgtDiv;

        for( int j = 1; j <= hgtDiv; ++j ){  segCtr.Add( btm + axis * j *  dHgt );  }

        for( int i = 1; i <= arcDiv; ++i ){
            // Rotate radial spar
            spar = MathVec3.AxisAngleQuat( axis, dTheta ) * lSpr;
            // Generate Top `Tri`
            rtnShp.Add( new Tri( top + lSpr, top, top + spar ) );
            // Generate Bottom `Tri`
            rtnShp.Add( new Tri( btm + spar, btm, btm + lSpr ) );
            // Generate longitudinal quad strip
            for( int j = 1; j <= hgtDiv; ++j ){
                Console.WriteLine( $"i: {i}, j: {j}" );
                p0 = segCtr[j-1] + lSpr;
                p1 = segCtr[j-1] + spar;
                p2 = segCtr[j  ] + lSpr;
                p3 = segCtr[j  ] + spar;
                rtnShp.Add( new Tri( p1, p0, p2 ) );
                rtnShp.Add( new Tri( p1, p2, p3 ) );
            }
            lSpr = spar;
        }
        return rtnShp;
    }


    /// <summary>
    /// Return a cylindrical mesh with a specified longitudinal axis
    /// </summary>
    public List<Tri> CircFrustum( Vector3 center, Vector3 axis, 
                                  float radius1 = 1f, float radius2 = 1f, float height = 1f, 
                                  int arcDiv = 16, int hgtDiv = 2 ){
        List<Tri>     rtnShp = [];
        List<Vector3> segCtr = [];
        rtnShp.Capacity = 2 * arcDiv + 2 * arcDiv * hgtDiv; // 2x Endcaps + `arcDiv` x `hgtDiv` Quads
        segCtr.Capacity = hgtDiv + 1;
        axis.Normalize();
        Vector3 top = center + axis * (height * 0.5f);
        Vector3 btm = center - axis * (height * 0.5f);
        segCtr.Add( btm );
        Vector3 unit = Vector3.Cross( axis, MathVec3.NoiseXYZ( rand ) ).Normalized();
        Vector3 p0, p1, p2, p3;
        Vector3 lUnt = unit;
        float   dTheta = 2f * MathF.PI / arcDiv;
        float   dHgt   = height / hgtDiv;
        float   radius_i;
        float   radius_l = radius1;
        float   radStep = (radius1 - radius2)/hgtDiv;

        for( int j = 1; j <= hgtDiv; ++j ){  segCtr.Add( btm + axis * j *  dHgt );  }

        for( int i = 1; i <= arcDiv; ++i ){
            
            // Rotate radial spar
            unit = MathVec3.AxisAngleQuat( axis, dTheta ) * lUnt;
            // Generate Top `Tri`
            rtnShp.Add( new Tri( top + lUnt*radius1, top, top + unit*radius1 ) );
            // Generate Bottom `Tri`
            rtnShp.Add( new Tri( btm + unit*radius2, btm, btm + lUnt*radius2 ) );
            // Generate longitudinal quad strip
            radius_l = radius2;
            for( int j = 1; j <= hgtDiv; ++j ){
                radius_i = radius2 + j * radStep;
                Console.WriteLine( $"i: {i}, j: {j}" );
                p0 = segCtr[j-1] + lUnt * radius_l;
                p1 = segCtr[j-1] + unit * radius_l;
                p2 = segCtr[j  ] + unit * radius_i;
                p3 = segCtr[j  ] + lUnt * radius_i;
                rtnShp.Add( new Tri( p1, p0, p2 ) );
                rtnShp.Add( new Tri( p2, p0, p3 ) );
                radius_l = radius_i;
                
            }
            lUnt = unit;
            
        }
        return rtnShp;
    }

}



/// <summary>
/// Flat face of a larger model
/// </summary>
public class Face {
    public List<Tri>     mesh /**/ = []; // Triangles that make up a flat face
    public List<Vector3> perimeter = []; // Cycle of vertices that define the edges of the face
}



/// <summary>
/// Directed edge between components
/// </summary>
public class Edge {
    public Matrix4 /*-*/ port = new();
    public IdeoComponent node = new();
}



/// <summary>
/// Composable 3D model with specified connection points
/// </summary>
public class IdeoComponent {
    public List<Tri>  mesh  = []; // Displayed mesh
    public List<Face> faces = []; // Flat faces for relative placement of components
    public List<Edge> edges = []; // Component neighbors
}

}
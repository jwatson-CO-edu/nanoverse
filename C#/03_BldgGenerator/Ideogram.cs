using OpenTK.Mathematics;
using geo3d;
using pose3d;
using parametric;

namespace ideogram {


////////////////////////////////////////////////////////////////////////////////////////////////////
////////// MESH GENERATION /////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


/// <summary>
/// Generate meshes suitable for composable models
/// </summary>
public class MeshGen {

    public Random rand = new(); // Local RNG


    /// <summary>
    /// Return a flat recilinear mesh in the XY plane
    /// </summary>
    public static List<Tri> PlaneXY( Vector3 center, float Xlength = 1f, float Ylength = 1f, float unit = 0f ) {
        if( unit > Math.Min( Xlength, Ylength ) ){  unit = 0f;  }
        if( unit < MathVec3._EPSILON ){  unit = Math.Min( Xlength, Ylength ) / 8f;  }
        List<Tri> rtnShp = [];

        Vector3 start = new( center[0] - Xlength/2f, center[1] - Xlength/2f, center[2] );
        int     Nx    = (int) (Xlength / unit) + 1;
        int     Ny    = (int) (Ylength / unit) + 1;
        rtnShp.Capacity = Nx * Ny;
        float   x0, y0, x1, y1;
        Vector3 p0, p1, p2, p3;

        for( int i = 0; i < Nx-1; ++i ){
            x0 = start[0] + Math.Min( i     * unit, Xlength );
            x1 = start[0] + Math.Min( (i+1) * unit, Xlength );
            for( int j = 0; j < Ny-1; ++j ){
                y0 = start[1] + Math.Min( j     * unit, Ylength );
                y1 = start[1] + Math.Min( (j+1) * unit, Ylength );
                p0 = new( x0, y0, center[2] );
                p1 = new( x1, y0, center[2] );
                p2 = new( x1, y1, center[2] );
                p3 = new( x0, y1, center[2] );
                rtnShp.Add( new Tri( p1, p0, p2 ) );
                rtnShp.Add( new Tri( p3, p2, p0 ) );
            }               
        }

        return rtnShp;
    }


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
        rtnShp.Add( new Tri( p0, p2, p1 ) );
        rtnShp.Add( new Tri( p2, p0, p3 ) );

        // Top //
        rtnShp.Add( new Tri( p4, p6, p5 ) );
        rtnShp.Add( new Tri( p6, p4, p7 ) );

        // Front //
        rtnShp.Add( new Tri( p0, p7, p4 ) );
        rtnShp.Add( new Tri( p7, p0, p1 ) );

        // Back //
        rtnShp.Add( new Tri( p2, p5, p6 ) );
        rtnShp.Add( new Tri( p5, p2, p3 ) );

        // Right //
        rtnShp.Add( new Tri( p4, p3, p0 ) );
        rtnShp.Add( new Tri( p3, p4, p5 ) );
        
        // Left //
        rtnShp.Add( new Tri( p1, p6, p7 ) );
        rtnShp.Add( new Tri( p6, p1, p2 ) );

        return rtnShp;
    }


    /// <summary>
    /// Return a cuboid mesh with arbitrary axes
    /// </summary>
    public static List<Tri> CuboidSpar( Vector3 center, Vector3 axis, Vector3 xDir, float Xside = 1f, float Yside = 1f, float Zside = 1f ){
        List<Tri> rtnShp = [];
        float     xHalf  = Xside / 2f;
        float     yHalf  = Yside / 2f;
        float     zHalf  = Zside / 2f;
        rtnShp.Capacity = 12;

        Matrix4 frame = MathMatx4.HomogFromXZBases( xDir, axis, center );
        Vector3 unitX = MathMatx4.GetXBasis( frame );
        Vector3 unitY = MathMatx4.GetYBasis( frame );
        Vector3 unitZ = MathMatx4.GetZBasis( frame );

        Vector3 p0 = center - unitX * xHalf - unitY * yHalf - unitZ * zHalf;
        Vector3 p1 = center - unitX * xHalf + unitY * yHalf - unitZ * zHalf;
        Vector3 p2 = center + unitX * xHalf + unitY * yHalf - unitZ * zHalf;
        Vector3 p3 = center + unitX * xHalf - unitY * yHalf - unitZ * zHalf;
        Vector3 p4 = center - unitX * xHalf - unitY * yHalf + unitZ * zHalf;
        Vector3 p5 = center + unitX * xHalf - unitY * yHalf + unitZ * zHalf;
        Vector3 p6 = center + unitX * xHalf + unitY * yHalf + unitZ * zHalf;
        Vector3 p7 = center - unitX * xHalf + unitY * yHalf + unitZ * zHalf;
        
        /*   6 ---- 5
           / |    / |
         7 ---- 4   |
         |   2 -|-- 3
         | /    | / 
         1 ---- 0   */
        
        // Bottom //
        rtnShp.Add( new Tri( p0, p2, p1 ) );
        rtnShp.Add( new Tri( p2, p0, p3 ) );

        // Top //
        rtnShp.Add( new Tri( p4, p6, p5 ) );
        rtnShp.Add( new Tri( p6, p4, p7 ) );

        // Front //
        rtnShp.Add( new Tri( p0, p7, p4 ) );
        rtnShp.Add( new Tri( p7, p0, p1 ) );

        // Back //
        rtnShp.Add( new Tri( p2, p5, p6 ) );
        rtnShp.Add( new Tri( p5, p2, p3 ) );

        // Right //
        rtnShp.Add( new Tri( p4, p3, p0 ) );
        rtnShp.Add( new Tri( p3, p4, p5 ) );
        
        // Left //
        rtnShp.Add( new Tri( p1, p6, p7 ) );
        rtnShp.Add( new Tri( p6, p1, p2 ) );

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
                // Console.WriteLine( $"i: {i}, j: {j}" );
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
    /// Return a truncated conical mesh with a specified longitudinal axis
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
        float   radius_i, radius_l;
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


    /// <summary>
    /// Return an icosahedron mesh
    /// </summary>
    public static List<Tri> Icosahedron( Vector3 cntr, float rad ) {
        // Compute the vertices and faces
        List<Tri> rtnShp = [];
        rtnShp.Capacity = 20;

        // ~ Constants ~
        float sqrt5 = MathF.Sqrt( 5.0f ); // ------------------------------------ Square root of 5
        float phi   = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------------- The Golden Ratio
        float ratio = MathF.Sqrt( 10.0f + ( 2.0f * sqrt5 ) ) / ( 4.0f * phi ); // ratio of edge length to radius
        float a     =  rad / ratio  * 0.5f;
        float b     =  rad / ratio  / ( 2.0f * phi );

        List<Vector3> V = [];
        V.Capacity = 12;

        // Define the icosahedron's 12 vertices:
        V.Add( new Vector3(  0,  b, -a ) + cntr );
        V.Add( new Vector3(  b,  a,  0 ) + cntr );
        V.Add( new Vector3( -b,  a,  0 ) + cntr );
        V.Add( new Vector3(  0,  b,  a ) + cntr );
        V.Add( new Vector3(  0, -b,  a ) + cntr );
        V.Add( new Vector3( -a,  0,  b ) + cntr );
        V.Add( new Vector3(  0, -b, -a ) + cntr );
        V.Add( new Vector3(  a,  0, -b ) + cntr );
        V.Add( new Vector3(  a,  0,  b ) + cntr );
        V.Add( new Vector3( -a,  0, -b ) + cntr );
        V.Add( new Vector3(  b, -a,  0 ) + cntr );
        V.Add( new Vector3( -b, -a,  0 ) + cntr );

        // Define the icosahedron's 20 triangular faces: CCW-out
        rtnShp.Add( new Tri( V[ 2], V[ 0], V[ 1] ) );
        rtnShp.Add( new Tri( V[ 1], V[ 3], V[ 2] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 3], V[ 4] ) );
        rtnShp.Add( new Tri( V[ 4], V[ 3], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 7], V[ 0], V[ 6] ) );
        rtnShp.Add( new Tri( V[ 6], V[ 0], V[ 9] ) );
        rtnShp.Add( new Tri( V[11], V[ 4], V[10] ) );
        rtnShp.Add( new Tri( V[10], V[ 6], V[11] ) );
        rtnShp.Add( new Tri( V[ 9], V[ 2], V[ 5] ) );
        rtnShp.Add( new Tri( V[ 5], V[11], V[ 9] ) );
        rtnShp.Add( new Tri( V[ 8], V[ 1], V[ 7] ) );
        rtnShp.Add( new Tri( V[ 7], V[10], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 2], V[ 3], V[ 5] ) );
        rtnShp.Add( new Tri( V[ 8], V[ 3], V[ 1] ) );
        rtnShp.Add( new Tri( V[ 9], V[ 0], V[ 2] ) );
        rtnShp.Add( new Tri( V[ 1], V[ 0], V[ 7] ) );
        rtnShp.Add( new Tri( V[11], V[ 6], V[ 9] ) );
        rtnShp.Add( new Tri( V[ 7], V[ 6], V[10] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 4], V[11] ) );
        rtnShp.Add( new Tri( V[10], V[ 4], V[ 8] ) );

        return rtnShp;
    }


    /// <summary>
    /// Return an tetrahedron mesh,
    /// Source: https://github.com/thinks/platonic-solids/blob/master/thinks/platonic_solids/platonic_solids.h
    /// </summary>
    public static List<Tri> Tetrahedron( Vector3 center, float height ) {
        List<Tri> rtnShp = [];
        rtnShp.Capacity = 4;

        // choose coordinates on the unit sphere
        float a = 1.0f / 3.0f;
        float b = MathF.Sqrt( 8.0f / 9.0f );
        float c = MathF.Sqrt( 2.0f / 9.0f );
        float d = MathF.Sqrt( 2.0f / 3.0f );

        // add the 4 vertices
        Vector3 v0 = new(  0,  0,  1 );
        Vector3 v1 = new( -c,  d, -a );
        Vector3 v2 = new( -c, -d, -a );
        Vector3 v3 = new(  b,  0, -a );
        v0 = v0 * height + center;
        v1 = v1 * height + center;
        v2 = v2 * height + center;
        v3 = v3 * height + center;

        // add the 4 faces
        rtnShp.Add( new Tri( v0, v2, v1 ) );
        rtnShp.Add( new Tri( v0, v3, v2 ) );
        rtnShp.Add( new Tri( v0, v1, v3 ) );
        rtnShp.Add( new Tri( v3, v1, v2 ) );

        return rtnShp;
    }


    /// <summary>
    /// Return an octahedron mesh,
    /// Source: https://github.com/thinks/platonic-solids/blob/master/thinks/platonic_solids/platonic_solids.h
    /// </summary>
    public static List<Tri> Octahedron( Vector3 center, float height ) {
        List<Tri> rtnShp = [];
        rtnShp.Capacity = 8;

        List<Vector3> V = [];
        V.Capacity = 6;

        V.Add( new Vector3(  0,  0, -1 ) );
        V.Add( new Vector3( -1,  0,  0 ) );
        V.Add( new Vector3(  0,  1,  0 ) );
        V.Add( new Vector3(  1,  0,  0 ) );
        V.Add( new Vector3(  0, -1,  0 ) );
        V.Add( new Vector3(  0,  0,  1 ) );

        for( int i = 0; i < V.Count; ++i ){  V[i] = V[i] * (height/2) + center;  }

        rtnShp.Add( new Tri( V[0], V[2], V[1] ) );
        rtnShp.Add( new Tri( V[3], V[2], V[0] ) );
        rtnShp.Add( new Tri( V[1], V[4], V[0] ) );
        rtnShp.Add( new Tri( V[2], V[5], V[1] ) );
        rtnShp.Add( new Tri( V[3], V[5], V[2] ) );
        rtnShp.Add( new Tri( V[4], V[3], V[0] ) );
        rtnShp.Add( new Tri( V[5], V[4], V[1] ) );
        rtnShp.Add( new Tri( V[5], V[3], V[4] ) );     

        return rtnShp;
    }


    /// <summary>
    /// Return an dodecahedron mesh,
    /// Source: https://github.com/thinks/platonic-solids/blob/master/thinks/platonic_solids/platonic_solids.h
    /// </summary>
    public static List<Tri> Dodecahedron( Vector3 center, float height ) {
        List<Tri> rtnShp = [];
        rtnShp.Capacity = 36;
        
        float sqrt5 = MathF.Sqrt( 5.0f ); // ------------------------------------ Square root of 5
        float p     = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------------- The Golden Ratio
        float ip    = 1f / p;

        List<Vector3> V = [];
        V.Capacity = 20;
    
        V.Add( new Vector3( -1 ,  1 , -1  ) );
        V.Add( new Vector3( -p ,  0 ,  ip ) );
        V.Add( new Vector3( -p ,  0 , -ip ) );
        V.Add( new Vector3( -1 ,  1 ,  1  ) );
        V.Add( new Vector3( -ip,  p ,  0  ) );
        V.Add( new Vector3(  1 ,  1 ,  1  ) );
        V.Add( new Vector3(  ip,  p ,  0  ) );
        V.Add( new Vector3(  0 ,  ip,  p  ) );
        V.Add( new Vector3( -1 , -1 ,  1  ) );
        V.Add( new Vector3(  0 , -ip,  p  ) );
        V.Add( new Vector3( -1 , -1 , -1  ) );
        V.Add( new Vector3( -ip, -p ,  0  ) );
        V.Add( new Vector3(  0 , -ip, -p  ) );
        V.Add( new Vector3(  0 ,  ip, -p  ) );
        V.Add( new Vector3(  1 ,  1 , -1  ) );
        V.Add( new Vector3(  p ,  0 , -ip ) );
        V.Add( new Vector3(  p ,  0 ,  ip ) );
        V.Add( new Vector3(  1 , -1 ,  1  ) );
        V.Add( new Vector3(  ip, -p ,  0  ) );
        V.Add( new Vector3(  1 , -1 , -1  ) );

        for( int i = 0; i < V.Count; ++i ){  V[i] = V[i] * (height/2) + center;  }

        rtnShp.Add( new Tri( V[ 1], V[ 2], V[ 0] ) );
        rtnShp.Add( new Tri( V[ 0], V[ 3], V[ 1] ) );
        rtnShp.Add( new Tri( V[ 0], V[ 4], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 4], V[ 6], V[ 5] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 3], V[ 4] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 7], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 8], V[ 1], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 3], V[ 7], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 7], V[ 9], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 8], V[11], V[10] ) );
        rtnShp.Add( new Tri( V[10], V[ 2], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 2], V[ 1], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 0], V[ 2], V[10] ) );
        rtnShp.Add( new Tri( V[10], V[12], V[ 0] ) );
        rtnShp.Add( new Tri( V[12], V[13], V[ 0] ) );
        rtnShp.Add( new Tri( V[ 0], V[13], V[14] ) );
        rtnShp.Add( new Tri( V[14], V[ 6], V[ 0] ) );
        rtnShp.Add( new Tri( V[ 6], V[ 4], V[ 0] ) );
        rtnShp.Add( new Tri( V[15], V[16], V[ 5] ) );
        rtnShp.Add( new Tri( V[ 5], V[14], V[15] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 6], V[14] ) );
        rtnShp.Add( new Tri( V[ 9], V[ 7], V[ 5] ) );
        rtnShp.Add( new Tri( V[ 5], V[17], V[ 9] ) );
        rtnShp.Add( new Tri( V[ 5], V[16], V[17] ) );
        rtnShp.Add( new Tri( V[18], V[11], V[ 8] ) );
        rtnShp.Add( new Tri( V[ 8], V[17], V[18] ) );
        rtnShp.Add( new Tri( V[ 8], V[ 9], V[17] ) );
        rtnShp.Add( new Tri( V[19], V[12], V[10] ) );
        rtnShp.Add( new Tri( V[10], V[11], V[19] ) );
        rtnShp.Add( new Tri( V[11], V[18], V[19] ) );
        rtnShp.Add( new Tri( V[13], V[12], V[19] ) );
        rtnShp.Add( new Tri( V[19], V[14], V[13] ) );
        rtnShp.Add( new Tri( V[19], V[15], V[14] ) );
        rtnShp.Add( new Tri( V[19], V[18], V[17] ) );
        rtnShp.Add( new Tri( V[17], V[16], V[19] ) );
        rtnShp.Add( new Tri( V[16], V[15], V[19] ) );
    
        return rtnShp;
    }


    /// <summary>
    /// Locate a 3D point within an arbitrary relative 2D basis
    /// </summary>
    public static Vector3 Vec3d_from_arbitrary_2D_basis( float x, float y, Vector3 xBasis, Vector3 yBasis ){
        // Return a coordinate in an arbitrary (non-orthoginal) 2D basis nested within a 3D frame
        // DO NOT normalize the basis vectors , see below!
        return xBasis * x + yBasis * y; 
    }


    /// <summary>
    /// Return a sphere (stellated icosahedron) mesh
    /// </summary>
    public static List<Tri> Sphere( float rad , Vector3 cntr, int div = 3 ) {
        // Compute the vertices and faces
        List<Tri> tris = [];
        tris.Capacity = 20 * (div*(div+1)/2 + (div-1)*div/2);
        List<Tri> icos = Icosahedron( cntr, rad );
        Vector3 v0, v1, v2, xTri, yTri, vA, vB, vC, nA, nB, nC;
        foreach( Tri tri in icos ){
            v0 = tri[0];  v1 = tri[1];  v2 = tri[2];
            xTri = ( v1 - v0 ) * (1.0f/div) ;
            yTri = ( v2 - v0 ) * (1.0f/div) ;

            for( int row = 1; row <= div; ++row ){
                for( int j = row ; j > 0 ; j-- ){ // Construct the v0-pointing tris
                    vA = v0 + Vec3d_from_arbitrary_2D_basis( j  , row-j  , xTri, yTri ) - cntr;
                    vB = v0 + Vec3d_from_arbitrary_2D_basis( j-1, row-j+1, xTri, yTri ) - cntr;
                    vC = v0 + Vec3d_from_arbitrary_2D_basis( j-1, row-j  , xTri, yTri ) - cntr;
                    nA = vA.Normalized();
                    nB = vB.Normalized();
                    nC = vC.Normalized();
                    vA = nA * rad + cntr;
                    vB = nB * rad + cntr;
                    vC = nC * rad + cntr;
                    tris.Add( new Tri( vA, vB, vC ) );
                }
                for( int j = row - 1 ; j > 0 ; j-- ){ // Construct the anti-v0-pointing tris
                    vA = v0 + Vec3d_from_arbitrary_2D_basis( j  , row-1-j  , xTri, yTri ) - cntr;
                    vB = v0 + Vec3d_from_arbitrary_2D_basis( j  , row-1-j+1, xTri, yTri ) - cntr;
                    vC = v0 + Vec3d_from_arbitrary_2D_basis( j-1, row-1-j+1, xTri, yTri ) - cntr;
                    nA = vA.Normalized();
                    nB = vB.Normalized();
                    nC = vC.Normalized();
                    vA = nA * rad + cntr;
                    vB = nB * rad + cntr;
                    vC = nC * rad + cntr;
                    tris.Add( new Tri( vA, vB, vC ) );
                }
            }
        }
        return tris;
    }


    /// <summary>
    /// Return an Elliptical Torus mesh in the XY plane
    /// </summary>
    public static List<Tri> EllipticalTorusXY( Vector3 cntr, float a, float b, float dia, int rotationRes = 32, int revolveRes = 16 ) {
        // Compute the vertices and faces
        List<Tri> tris = [];
        tris.Capacity = rotationRes * revolveRes * 2;

        Vector3 circCntr_i, circCntr_ip1, axis_i, axis_ip1;
        Vector3 rad_i, rad_ip1;
        Vector3 p1, p2, p3, p4;

        float theta   = 0.0f, phi;
        float rotStep = 2.0f * MathF.PI / (1.0f * rotationRes);
        float revStep = 2.0f * MathF.PI / (1.0f * revolveRes );

        for( uint i = 0; i < rotationRes; ++i ){ 

            circCntr_i   = new Vector3( a*MathF.Cos(theta)        , b*MathF.Sin(theta)        , 0.0f );
            circCntr_ip1 = new Vector3( a*MathF.Cos(theta+rotStep), b*MathF.Sin(theta+rotStep), 0.0f );
            axis_i /*-*/ = Vector3.Cross( new Vector3( 0.0f,0.0f,-1.0f ), circCntr_i   );
            axis_ip1     = Vector3.Cross( new Vector3( 0.0f,0.0f,-1.0f ), circCntr_ip1 );
            phi /*----*/ = 0.0f;
            rad_i /*--*/ = circCntr_i.Normalized()   * dia/2.0f;
            rad_ip1 /**/ = circCntr_ip1.Normalized() * dia/2.0f;

            for( uint j = 0; j < revolveRes; ++j ){

                p1 =  circCntr_i   + MathVec3.AxisAngleQuat( axis_i  , phi         ) * rad_i   + cntr; 
                p2 =  circCntr_ip1 + MathVec3.AxisAngleQuat( axis_ip1, phi         ) * rad_ip1 + cntr; 
                p3 =  circCntr_i   + MathVec3.AxisAngleQuat( axis_i  , phi+revStep ) * rad_i   + cntr; 
                p4 =  circCntr_ip1 + MathVec3.AxisAngleQuat( axis_ip1, phi+revStep ) * rad_ip1 + cntr; 

                tris.Add( new Tri(p3, p2, p1) );
                tris.Add( new Tri(p3, p4, p2) );

                phi += revStep;
            }
            theta += rotStep;
        }

        return tris;
    }


    /// <summary>
    /// Return a wedge with a rounded back
    /// </summary>
    public static List<Tri> Wedge( Vector3 center, Vector3 axis, Vector3 begin, 
                                   float arcTheta, float height, float width, int div = 16 ){
        axis.Normalize();
        List<Tri> tris = [];
        tris.Capacity = div * 4 + 4;
        float   half   = height/2f;
        float   dTheta = arcTheta / div;
        Matrix4 frame  = MathMatx4.HomogFromXZBases( begin.Normalized(), axis.Normalized(), center );
        Vector3 xDir   = MathMatx4.GetXBasis( frame );
        Vector3 top    = center + axis * half;
        Vector3 btm    = center - axis * half;
        Vector3 spar   = xDir * width;
        Vector3 rib1, rib2, p0, p1, p2, p3;

        /// Begining Face ///
        p0 = top;
        p1 = btm;
        p2 = btm + spar;
        p3 = top + spar;
        tris.Add( new Tri( p1, p0, p2 ) );
        tris.Add( new Tri( p2, p0, p3 ) );

        /// Arc ///
        for( int i = 0; i < div; ++i ){
            rib1 = MathVec3.AxisAngleQuat( axis, dTheta * i     ) * spar;
            rib2 = MathVec3.AxisAngleQuat( axis, dTheta * (i+1) ) * spar;
            p0   = btm + rib1;
            p1   = top + rib1;
            p2   = top + rib2;
            p3   = btm + rib2;
            tris.Add( new Tri( top, p2, p1 ) ); // Top Pie Slice
            tris.Add( new Tri( p1 , p2, p0 ) ); // Outer 1/2
            tris.Add( new Tri( p2 , p3, p0 ) ); // Outer 2/2
            tris.Add( new Tri( btm, p0, p3 ) ); // Bottom Pie Slice
        }

        /// Ending Face ///
        rib2 = MathVec3.AxisAngleQuat( axis, dTheta * div ) * spar;
        p0   = btm;
        p1   = top;
        p2   = top + rib2;
        p3   = btm + rib2;
        tris.Add( new Tri( p1, p0, p2 ) );
        tris.Add( new Tri( p2, p0, p3 ) );

        return tris;
    }


    /// <summary>
    /// Return an arc with a rectangular profile
    /// </summary>
    public static List<Tri> Arc( Vector3 center, Vector3 axis, Vector3 begin, 
                                 float arcTheta, float height, float radInner, float radOuter, int div = 16 ){
        axis.Normalize();
        List<Tri> tris = [];
        tris.Capacity = div * 8 + 4;
        float   half   = height/2f;
        float   dTheta = arcTheta / div;
        Matrix4 frame  = MathMatx4.HomogFromXZBases( begin.Normalized(), axis.Normalized(), center );
        Vector3 xDir   = MathMatx4.GetXBasis( frame );
        Vector3 top    = center + axis * half;
        Vector3 btm    = center - axis * half;
        Vector3 sparInner   = xDir * radInner;
        Vector3 sparOuter   = xDir * radOuter;
        Vector3 rib1, rib2, rib3, rib4, p0, p1, p2, p3, p4, p5, p6, p7;

        /// Begining Face ///
        p0 = top + sparInner;
        p1 = btm + sparInner;
        p2 = btm + sparOuter;
        p3 = top + sparOuter;
        tris.Add( new Tri( p1, p0, p2 ) );
        tris.Add( new Tri( p2, p0, p3 ) );

        /// Arc ///
        for( int i = 0; i < div; ++i ){
            rib1 = MathVec3.AxisAngleQuat( axis, dTheta * i     ) * sparInner;
            rib2 = MathVec3.AxisAngleQuat( axis, dTheta * (i+1) ) * sparInner;
            rib3 = MathVec3.AxisAngleQuat( axis, dTheta * i     ) * sparOuter;
            rib4 = MathVec3.AxisAngleQuat( axis, dTheta * (i+1) ) * sparOuter;
            p0   = btm + rib1;
            p1   = top + rib1;
            p2   = top + rib2;
            p3   = btm + rib2;
            p4   = btm + rib3;
            p5   = top + rib3;
            p6   = top + rib4;
            p7   = btm + rib4;

            tris.Add( new Tri( p5, p1, p6 ) ); // Top 1/2
            tris.Add( new Tri( p6, p1, p2 ) ); // Top 2/2
            
            tris.Add( new Tri( p1 , p2, p0 ) ); // Outer 1/2
            tris.Add( new Tri( p2 , p3, p0 ) ); // Outer 2/2

            tris.Add( new Tri( p5, p6, p4 ) ); // Inner 1/2
            tris.Add( new Tri( p6, p7, p4 ) ); // Inner 2/2
            
            tris.Add( new Tri( p4, p7, p0 ) ); // Bottom 1/2
            tris.Add( new Tri( p7, p3, p0 ) ); // Bottom 2/2
        }

        /// Ending Face ///
        rib1 = MathVec3.AxisAngleQuat( axis, dTheta * div ) * sparInner;
        rib2 = MathVec3.AxisAngleQuat( axis, dTheta * div ) * sparOuter;
        p0   = btm + rib1;
        p1   = top + rib1;
        p2   = top + rib2;
        p3   = btm + rib2;
        tris.Add( new Tri( p1, p0, p2 ) );
        tris.Add( new Tri( p2, p0, p3 ) );

        return tris;
    }


    /// <summary>
    /// Return an inclined plane
    /// </summary>
    public static List<Tri> Incline( Vector3 center, float xLen, float zHeight, float yWidth ){
        List<Tri> rtnShp  = [];
        rtnShp.Capacity = 8;

        Vector3 xBar = Vector3.UnitX * xLen;
        Vector3 zBar = Vector3.UnitZ * zHeight;
        Vector3 yHlf = Vector3.UnitY * (yWidth/2f);
        Vector3 p0, p1, p2, p3;
        
        p0 = center + yHlf;
        p1 = center + yHlf + xBar;
        p2 = center - yHlf + xBar;
        p3 = center - yHlf;
        rtnShp.Add( new Tri( p1, p0, p2 ) );
        rtnShp.Add( new Tri( p2, p0, p3 ) );

        p1 = center + yHlf + zBar;
        p2 = center - yHlf + zBar;
        rtnShp.Add( new Tri( p1, p2, p0 ) );
        rtnShp.Add( new Tri( p2, p3, p0 ) );

        p0 = center + yHlf+ xBar;
        p3 = center - yHlf+ xBar;
        rtnShp.Add( new Tri( p1, p0, p2 ) );
        rtnShp.Add( new Tri( p2, p0, p3 ) );

        p0 = center + yHlf;
        p1 = center + yHlf + xBar;
        p2 = center + yHlf + zBar;
        rtnShp.Add( new Tri( p0, p1, p2 ) );
        
        p0 = center - yHlf;
        p1 = center - yHlf + zBar;
        p2 = center - yHlf + xBar;
        rtnShp.Add( new Tri( p0, p1, p2 ) );

        return rtnShp;
    }


    /// <summary>
    /// Return a frustrum with a profile that rotates as it moves from bottom to top
    /// </summary>
    public static List<Tri> TwistFrustum( Vector3 center, Vector3 axis, Vector3 begin, float twist = 0f,
                                          float radius1 = 1f, float radius2 = 1f, float height = 1f, 
                                          int arcDiv = 4, int hgtDiv = 16 ){
        axis.Normalize();
        List<Tri> /*--*/ rtnShp  = [];
        Matrix4 /*----*/ frame   = MathMatx4.HomogFromXZBases( begin.Normalized(), axis.Normalized(), center );
        Vector3 /*----*/ xDir    = MathMatx4.GetXBasis( frame );
        float /*------*/ half    = height/2f;
        float /*------*/ dt /**/ = 1f/hgtDiv;
        float /*------*/ dTheta  = 2f*MathF.PI/arcDiv;
        Vector3 /*----*/ top     = center + axis * half;
        Vector3 /*----*/ btm     = center - axis * half;
        List<Vector3>    btmRing = [];
        List<Vector3>    topRing = [];
        List<Vector3>    P0 /**/ = [];
        List<Vector3>    P1 /**/ = [];
        List<Vector3>    P2 /**/ = [];
        List<Vector3>    P3 /**/ = [];
        Vector3 /*----*/ spar, btm_i, top_i, temp, lTop, lBtm, pnt0, pnt1, pnt2, pnt3;
        float /*------*/ ctrlDiv = 6f, t_i, t_ip1;

        /// Construct Ribs ///
        for( int i = 0; i < arcDiv; ++i ){
            spar = MathVec3.AxisAngleQuat( axis, i*dTheta ) * xDir;
            // 1. Bottom ring
            btmRing.Add( btm + spar * radius2 );
            // 2. Top ring (Pre-rotation)
            topRing.Add( top + spar * radius1 );
            btm_i = btmRing[^1];
            top_i = topRing[^1];
            // Bottom to Top //
            P0.Add( btm_i );
            // 3. Inner control points
            P1.Add( btm_i + (top_i - btm_i) / ctrlDiv );
            // Top to Bottom (Relative) //
            P2.Add( (btm_i - top_i) / ctrlDiv );
        }

        for( int i = 0; i < arcDiv; ++i ){
    
            // 4. Rotated Top ring 
            spar = MathVec3.AxisAngleQuat( axis, i*dTheta+twist ) * xDir;
            temp = top + spar * radius1;
            topRing[i] = temp;
            P2[i] += temp;
            P3.Add( temp );
        }
        
        lTop = topRing[^1];
        lBtm = btmRing[^1];
        for( int i = 0; i < arcDiv; ++i ){
            /// Top Face ///
            rtnShp.Add( new Tri( top, topRing[i], lTop ) );
            lTop = topRing[i];
            
            /// Bottom Face ///    
            rtnShp.Add( new Tri( btm, lBtm, btmRing[i] ) );
            lBtm = btmRing[i];
    
        }

        /// Twisted Sides ///
        for( int i = 0; i < hgtDiv; ++i ){
            t_i   = i * dt;
            t_ip1 = (i+1) * dt;
            for( int j = 0; j < arcDiv; ++j ){
                pnt0 = Bezier.Cubic.Value( P0[j  ], P1[j  ], P2[j  ], P3[j  ], t_i   );
                pnt1 = Bezier.Cubic.Value( P0[(j+1)%arcDiv], P1[(j+1)%arcDiv], P2[(j+1)%arcDiv], P3[(j+1)%arcDiv], t_i   );
                pnt2 = Bezier.Cubic.Value( P0[(j+1)%arcDiv], P1[(j+1)%arcDiv], P2[(j+1)%arcDiv], P3[(j+1)%arcDiv], t_ip1 );
                pnt3 = Bezier.Cubic.Value( P0[j  ], P1[j  ], P2[j  ], P3[j  ], t_ip1 );
                rtnShp.Add( new Tri( pnt1, pnt0, pnt2 ) );
                rtnShp.Add( new Tri( pnt2, pnt0, pnt3 ) );
            }
        }

        return rtnShp;
    }

}



////////////////////////////////////////////////////////////////////////////////////////////////////
////////// COMPOSITE MODELS ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


// /// <summary>
// /// Flat face of a larger model
// /// </summary>
// public class Face {
//     public List<Tri>     mesh /**/ = []; // Triangles that make up a flat face
//     public List<Vector3> perimeter = []; // Cycle of vertices that define the edges of the face
// }



/// <summary>
/// Composable 3D model with specified connection points
/// </summary>
public class Node {
    public Matrix4 /*-------------*/ pose  = new(); // Location + Orientation in space
    public List<Tri> /*-----------*/ mesh  = []; // -- Displayed mesh
    public List<Matrix4> /*-------*/ ports = []; // -- Relative  
    public List<Node> /*----------*/ edges = []; // -- Component neighbors
    public Dictionary<string,string> tags  = []; // -- Categorical Descriptors
    public Dictionary<string,float>  attrs = []; // -- Numeric Attributes   
}

}
using geo3d;
using OpenTK.Mathematics;

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
    public static List<Tri> Icosahedron( float rad , Vector3 cntr ) {
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
        rtnShp.Add( new Tri( V[ 2], V[ 1], V[ 0] ) );
        rtnShp.Add( new Tri( V[ 1], V[ 2], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 4], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 4], V[ 8], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 7], V[ 6], V[ 0] ) );
        rtnShp.Add( new Tri( V[ 6], V[ 9], V[ 0] ) );
        rtnShp.Add( new Tri( V[11], V[10], V[ 4] ) );
        rtnShp.Add( new Tri( V[10], V[11], V[ 6] ) );
        rtnShp.Add( new Tri( V[ 9], V[ 5], V[ 2] ) );
        rtnShp.Add( new Tri( V[ 5], V[ 9], V[11] ) );
        rtnShp.Add( new Tri( V[ 8], V[ 7], V[ 1] ) );
        rtnShp.Add( new Tri( V[ 7], V[ 8], V[10] ) );
        rtnShp.Add( new Tri( V[ 2], V[ 5], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 8], V[ 1], V[ 3] ) );
        rtnShp.Add( new Tri( V[ 9], V[ 2], V[ 0] ) );
        rtnShp.Add( new Tri( V[ 1], V[ 7], V[ 0] ) );
        rtnShp.Add( new Tri( V[11], V[ 9], V[ 6] ) );
        rtnShp.Add( new Tri( V[ 7], V[10], V[ 6] ) );
        rtnShp.Add( new Tri( V[ 5], V[11], V[ 4] ) );
        rtnShp.Add( new Tri( V[10], V[ 8], V[ 4] ) );

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
        List<Tri> icos = Icosahedron( rad, cntr );
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

                tris.Add( new Tri(p3, p1, p2) );
                tris.Add( new Tri(p3, p2, p4) );

                phi += revStep;
            }
            theta += rotStep;
        }

        return tris;
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
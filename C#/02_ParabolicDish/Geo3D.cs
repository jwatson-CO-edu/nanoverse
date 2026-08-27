using OpenTK.Mathematics;
using System.Text;

using MathNet.Numerics.LinearAlgebra;



/// <summary>
/// Helper functions for 3D Geometry
/// </summary>
namespace geo3d {



////////// VECTOR3 MATH ////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Working with `OpenTK.Mathematics.Vector3`
/// </summary>
public static class MathVec3 {

    const float _EPSILON = 0.00001f; // Error margin, 1/100 [mm], We don't care about anything smaller!

    
    /// <summary>
    /// Are the vectors equal?
    /// </summary>
    public static bool Equal( Vector3 vec1 , Vector3 vec2 ){  return (vec1 - vec2).Length < _EPSILON;  }


    /// <summary>
    /// Get the angle between two R3 vectors , radians
    /// </summary>
    public static float AngleBetween( Vector3 vec1 , Vector3 vec2 ){
        float angle = (float) Math.Acos( Vector3.Dot( vec1.Normalized(), vec2.Normalized() ) ); // for now assume that there are no special cases
        if( float.IsNaN( angle ) ){
            if( Equal( vec1.Normalized(), vec2.Normalized() ) ){ return 0.0f; }
            else { return (float) Math.PI; }
        } else { return angle; }
    }


    /// <summary>
    /// `Quaternion.FromAxisAngle`, enforce normalized axis
    /// </summary>
    public static Quaternion AxisAngleQuat( Vector3 axis, float theta ){
        return Quaternion.FromAxisAngle( axis.Normalized(), theta );
    }


    /// <summary>
    /// Generate a 3D vector with random {X,Y,} components of a specified `scale`
    /// </summary>
    public static Vector3 NoiseXY( Random rand, float scale = 1.0f ){
        return new Vector3( rand.NextSingle()-0.5f, rand.NextSingle()-0.5f, 0.0f ).Normalized() * scale;
    }


    /// <summary>
    /// Get the (uniformly weighted) centroid of the collection of points
    /// </summary>
    public static Vector3 UniformPointCentroid( IEnumerable<Vector3> points ){
        Vector3 rtnPnt = new(0,0,0);
        foreach( Vector3 pnt in points ){  rtnPnt += pnt;  }
        return rtnPnt / points.Count();
    }


    /// <summary>
    /// Return a shifted copy of the collection of points
    /// </summary>
    public static List<Vector3> ShiftPoints( IEnumerable<Vector3> points, Vector3 shift ){
        List<Vector3> rtnLst = [];
        rtnLst.Capacity = points.Count();
        foreach( Vector3 pnt in points ){  rtnLst.Add( pnt + shift);  }
        return rtnLst;
    }


    /// <summary>
    /// Return centers of the specified XY grid cells
    /// </summary>
    public static List<Vector3> GridCentersXY( float unit, int Nx, int Ny, bool center = true ){
        List<Vector3> rtnLst = [];
        rtnLst.Capacity = Nx * Ny;
        float half = unit / 2.0f;
        for( int i = 0; i < Nx; ++i ){  
            for( int j = 0; j < Ny; ++j ){
                rtnLst.Add( new Vector3( half + i*unit, half + j*unit, 0.0f ) );
            }               
        }
        if( center ){  return ShiftPoints( rtnLst, -UniformPointCentroid( rtnLst ) );  }
        return rtnLst;
    }


    /// <summary>
    /// Return the closest point between two line segments,
    /// Source: "wes", https://math.stackexchange.com/a/4289668
    /// </summary>
    public static Vector3 ClosestPointBetweenLineSegments( Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4 ){

        Vector3 P1  = p1;
        Vector3 P2  = p3;
        Vector3 V1  = p2 - p1;
        Vector3 V2  = p4 - p3;
        Vector3 V21 = P2 - P1;

        float v22   = Vector3.Dot( V2 , V2 );
        float v11   = Vector3.Dot( V1 , V1 );
        float v21   = Vector3.Dot( V2 , V1 );
        float v21_1 = Vector3.Dot( V21, V1 );
        float v21_2 = Vector3.Dot( V21, V2 );
        float denom = v21 * v21 - v22 * v11;
        float s, t;

        if( denom < 0.00001f ){
            s = 0f;
            t = (v11 * s - v21_1) / v21;
        }else{
            s = (v21_2 * v21 - v22 * v21_1) / denom;
            t = (-v21_1 * v21 + v11 * v21_2) / denom;
        }
        
        s = Math.Max( Math.Min( s, 1f ), 0f );
        t = Math.Max( Math.Min( t, 1f ), 0f );

        Vector3 p_a = P1 + s * V1;
        Vector3 p_b = P2 + t * V2;

        return (p_a + p_b)/2f;
    }


    /// <summary>
    /// Intepret a collection of OpenTK `Vector3` as a Math.NET matrix,
    /// SLOP: https://claude.ai/share/79b43aee-2f43-4161-bd11-2bbb725ed2db
    /// </summary>
    public static Matrix<double> ToMathNet( this IReadOnlyList<Vector3> points ){
        var m = Matrix<double>.Build.Dense( points.Count, 3 );
        for( int i = 0; i < points.Count; i++ ){
            m[i, 0] = points[i].X;
            m[i, 1] = points[i].Y;
            m[i, 2] = points[i].Z;
        }
        return m;
    }


    /// <summary>
    /// Interpret a Math.NET vector as an OpenTK `Vector3`,
    /// SLOP: https://claude.ai/share/79b43aee-2f43-4161-bd11-2bbb725ed2db
    /// </summary>
    public static Vector3 ToOpenTK( this Vector<double> v ){
        if( v.Count != 3 ){  throw new ArgumentException( "Vector must have exactly 3 elements." );  }
        return new Vector3( (float)v[0], (float)v[1], (float)v[2] );
    }


}



////////////////////////////////////////////////////////////////////////////////////////////////////
////////// GEOMETRIC STRUCTS ///////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////



////////// SEGMENTS ///////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Segment
/// </summary>
public readonly struct LinSeg{

    /// Members ///
    // Array refs cannot change, values can
    private readonly Vector3[] /*----------*/ verts = new Vector3[2]; // Vertex Positions 
    private readonly Vector4[] /*----------*/ color = new Vector4[2]; // Vertex Colors 
    public  readonly Dictionary<string,float> attrs = []; // ----------- Attributes
    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public LinSeg( Vector3 a, Vector3 b ){
        verts[0] = a;
        verts[1] = b;
    }


    /// <summary>
    /// Set uniform color for all vertices
    /// </summary>
    public void SetColor( Vector4 colr ){
        color[0] = colr;
        color[1] = colr;
    }


    /// <summary>
    /// Set uniform color for all vertices
    /// </summary>
    public void SetColor( Vector3 colr, float alpha = 1f ){
        color[0] = new Vector4( colr[0], colr[1], colr[2], alpha );
        color[1] = new Vector4( colr[0], colr[1], colr[2], alpha );
    }


    /// <summary>
    /// Set per vertex colors
    /// </summary>
    public void SetColor( Vector4 a, Vector4 b ){
        color[0] = a;
        color[1] = b;
    }


    /// <summary>
    /// Set per vertex colors
    /// </summary>
    public void SetColor( Vector3 a, Vector3 b, Vector3 c, float alpha = 1f ){
        color[0] = new Vector4( a[0], a[1], a[2], alpha );
        color[1] = new Vector4( b[0], b[1], b[2], alpha );
    }


    /// <summary>
    /// Set uniform opacity for all vertices
    /// </summary>
    public void SetAlpha( float alpha ){
        color[0][3] = alpha;
        color[1][3] = alpha;
    }


    /// <summary>
    /// First point (CCW)
    /// </summary>
    public readonly Vector3 V0() => verts[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector3 V1() => verts[1];


    /// <summary>
    /// Centroid of a collection of `Segment`s
    /// </summary>
    public static Vector3 Centroid( IEnumerable<LinSeg> cllctn ){
        Vector3 center = new(0,0,0);
        foreach( LinSeg seg in cllctn ){  center += seg.V0() + seg.V1();  }
        return center / (cllctn.Count() * 2);
    }


    /// <summary>
    /// Calculate the collection Axis-Aligned Bounding Box (AABB)
    /// </summary>
    public static Vector3[] BBox( IEnumerable<LinSeg> cllctn ){
        Vector3 lo = new(  6e10f,  6e10f,  6e10f );
        Vector3 hi = new( -6e10f, -6e10f, -6e10f );
        foreach( LinSeg seg in cllctn ){
            foreach( Vector3 pt in seg.verts ){
                for( int k = 0; k < 3; ++k ){
                    lo[k] = Math.Min( lo[k], pt[k] );
                    hi[k] = Math.Max( hi[k], pt[k] );
                }
            }
        }
        return [lo, hi,];
    }


    /// <summary>
    /// Return a shifted a collection of `Segment`s
    /// </summary>
    public static List<LinSeg> CopySegments( IEnumerable<LinSeg> cllctn ){
        List<LinSeg> rtnLst = [];
        LinSeg nuSeg;
        rtnLst.Capacity = cllctn.Count();
        foreach( LinSeg seg in cllctn ){
            nuSeg = new LinSeg( seg.V0(), seg.V1() );
            foreach( (string k, float v) in seg.attrs ){  nuSeg.attrs[k] = v;  }
            rtnLst.Add( nuSeg );
        }
        return rtnLst;
    }


    /// <summary>
    /// Return a shifted a collection of `Segment`s
    /// </summary>
    public static List<LinSeg> ShiftSegments( IEnumerable<LinSeg> cllctn, Vector3 shift ){
        List<LinSeg> rtnLst = CopySegments( cllctn );
        for( int i = 0; i < rtnLst.Count; ++i ){
            rtnLst[i].verts[0] += shift; 
            rtnLst[i].verts[1] += shift; 
        }
        return rtnLst;
    }


    /// <summary>
    /// Set a uniform weight for collection of `Segment`s
    /// </summary>
    public static void SetWeight( IEnumerable<LinSeg> cllctn, float weight ){
        foreach( LinSeg seg in cllctn ){  seg.attrs["weight"] = weight;  }
    }


    /// <summary>
    /// Return a rotated a collection of `Segment`s
    /// </summary>
    public static List<LinSeg> RotateSegments( IEnumerable<LinSeg> segments, Vector3 center, Vector3 axis, float theta ){
        List<LinSeg> rtnLst = ShiftSegments( segments, -center );
        Quaternion   quat   = MathVec3.AxisAngleQuat( axis, theta );
        for( int i = 0; i < rtnLst.Count; ++i ){
            rtnLst[i].verts[0] = quat * rtnLst[i].verts[0]; 
            rtnLst[i].verts[1] = quat * rtnLst[i].verts[1]; 
        }
        return ShiftSegments( rtnLst, center );
    }


    /// <summary>
    /// Do the `Segment`s overlap?
    /// </summary>
    public bool IsShared( LinSeg other ){
        if( MathVec3.Equal( V0(), other.V0() ) && MathVec3.Equal( V1(), other.V1() ) ){  return true;  }
        if( MathVec3.Equal( V0(), other.V1() ) && MathVec3.Equal( V1(), other.V0() ) ){  return true;  }
        return false;
    }


}



////////// TRIANGLES ///////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Triangle
/// </summary>
public readonly struct Tri{

    /// Members ///
    // Array refs cannot change, values can
    private readonly Vector3[] verts = new Vector3[3]; // Vertex Positions 
    private readonly Vector3[] norms = new Vector3[3]; // Norm Directions 
    private readonly Vector4[] color = new Vector4[3]; // Vertex Colors 

    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public Tri( Vector3 a, Vector3 b, Vector3 c ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
        Vector3 ab = V1() - V0();
        Vector3 ac = V2() - V0();
        SetNorms( Vector3.Cross( ac, ab ) );
    }


    /// <summary>
    /// Set uniform norm for all vertices
    /// </summary>
    public void SetNorms( Vector3 norm ){
        norm.Normalize();
        norms[0] = norm;
        norms[1] = norm;
        norms[2] = norm;
    }


    /// <summary>
    /// Set per vertex norms
    /// </summary>
    public void SetNorms( Vector3 a, Vector3 b, Vector3 c ){
        norms[0] = a.Normalized();
        norms[1] = b.Normalized();
        norms[2] = c.Normalized();
    }


    /// <summary>
    /// Set uniform color for all vertices
    /// </summary>
    public void SetColor( Vector4 colr ){
        color[0] = colr;
        color[1] = colr;
        color[2] = colr;
    }


    /// <summary>
    /// Set uniform color for all vertices
    /// </summary>
    public void SetColor( Vector3 colr, float alpha = 1f ){
        color[0] = new Vector4( colr[0], colr[1], colr[2], alpha );
        color[1] = new Vector4( colr[0], colr[1], colr[2], alpha );
        color[2] = new Vector4( colr[0], colr[1], colr[2], alpha );
    }


    /// <summary>
    /// Set per vertex colors
    /// </summary>
    public void SetColor( Vector4 a, Vector4 b, Vector4 c ){
        color[0] = a;
        color[1] = b;
        color[2] = c;
    }


    /// <summary>
    /// Set per vertex colors
    /// </summary>
    public void SetColor( Vector4 a, Vector3 b, Vector3 c, float alpha = 1f ){
        color[0] = new Vector4( a[0], a[1], a[2], alpha );
        color[1] = new Vector4( b[0], b[1], b[2], alpha );
        color[2] = new Vector4( c[0], c[1], c[2], alpha );
    }


    /// <summary>
    /// Set uniform opacity for all vertices
    /// </summary>
    public void SetAlpha( float alpha ){
        color[0][3] = alpha;
        color[1][3] = alpha;
        color[2][3] = alpha;
    }


    /// <summary>
    /// Centroid of the Triangle
    /// </summary>
    public Vector3 Center(){
        Vector3 rtnVec = new(0,0,0);
        foreach( Vector3 vec in verts ){  rtnVec += vec;  }
        return rtnVec / 3.0f;
    }


    /// <summary>
    /// Area of the Triangle
    /// </summary>
    public float Area(){
        Vector3 AB  = verts[1] - verts[0];
        Vector3 AC  = verts[2] - verts[0];
        float   mag = Vector3.Cross( AB, AC ).Length;
        return mag / 2.0f;
    }


    /// <summary>
    /// Return the area centroid of a collection of triangles
    /// </summary>
    public static float MeshArea( IEnumerable<Tri> mesh ){
        float   totArea  = 0.0f;
        foreach( Tri tri in mesh ){  totArea += tri.Area();  }
        return totArea;
    }


    /// <summary>
    /// Return the area centroid of a collection of triangles
    /// </summary>
    public static Vector3 MeshAreaCentroid( IEnumerable<Tri> mesh ){
        float   area_i;
        float   totArea  = 0.0f;
        Vector3 centroid = new(0,0,0);
        foreach( Tri tri in mesh ){
            area_i    = tri.Area();
            totArea  += area_i;
            centroid += tri.Center() * area_i;
        }
        return centroid / totArea;
    }


    /// <summary>
    /// Calculate the mesh Axis-Aligned Bounding Box (AABB)
    /// </summary>
    public static Vector3[] MeshBBox( IEnumerable<Tri> mesh ){
        Vector3 lo = new(  6e10f,  6e10f,  6e10f );
        Vector3 hi = new( -6e10f, -6e10f, -6e10f );
        foreach( Tri tri in mesh ){
            foreach( Vector3 pt in tri.verts ){
                for( int k = 0; k < 3; ++k ){
                    lo[k] = Math.Min( lo[k], pt[k] );
                    hi[k] = Math.Max( hi[k], pt[k] );
                }
            }
        }
        return [lo, hi,];
    }


    /// <summary>
    /// Return a shifted a collection of triangles
    /// </summary>
    public static List<Tri> ShiftMesh( IEnumerable<Tri> mesh, Vector3 shift ){
        Tri tri_i;
        List<Tri> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count();
        foreach( Tri tri in mesh ){
            tri_i = new( tri.V0() + shift, 
                         tri.V1() + shift, 
                         tri.V2() + shift );
            rtnMsh.Add( tri_i );
        }
        return rtnMsh;
    }


    /// <summary>
    /// Return a shifted a collection of triangles
    /// </summary>
    public static List<Tri> RotateMesh( IEnumerable<Tri> mesh, Vector3 axis, float theta ){
        Tri tri_i;
        List<Tri> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count();
        axis.Normalize();
        Quaternion quat = MathVec3.AxisAngleQuat( axis, theta );
        foreach( Tri tri in mesh ){
            tri_i = new( quat * tri.V0(), 
                         quat * tri.V1(), 
                         quat * tri.V2() );
            rtnMsh.Add( tri_i );
        }
        return rtnMsh;
    }


    /// <summary>
    /// Copy the entire mesh
    /// </summary>
    public static List<Tri> CopyMesh( IEnumerable<Tri> mesh ){
        List<Tri> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count();
        foreach( Tri tri in mesh ){
            rtnMsh.Add( new Tri( tri.V0(), 
                                 tri.V1(), 
                                 tri.V2() ) );
        }
        return rtnMsh;
    }


    /// <summary>
    /// Apply one color to the entire mesh
    /// </summary>
    public static List<Tri> ColorMesh( IEnumerable<Tri> mesh, Vector4 color ){
        List<Tri> rtnMsh = CopyMesh( mesh );
        foreach( Tri tri in rtnMsh ){  tri.SetColor( color );  }
        return rtnMsh;
    }


    /// <summary>
    /// Indexer: {get, set,}
    /// </summary>
    public readonly Vector3 this[int index]{
        get => verts[ index ];
        set => verts[ index ] = value;
    }


    /// <summary>
    /// First point (CCW)
    /// </summary>
    public readonly Vector3 V0() => verts[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector3 V1() => verts[1];
    
    
    /// <summary>
    /// Third point (CCW)
    /// </summary>
    public readonly Vector3 V2() => verts[2];


    /// <summary>
    /// Alt vertex accessor (CCW)
    /// </summary>
    public readonly Vector3 V( int index ) => verts[ index ];


    /// <summary>
    /// First normal (CCW)
    /// </summary>
    public readonly Vector3 N0() => norms[0];
    

    /// <summary>
    /// Second normal (CCW)
    /// </summary>
    public readonly Vector3 N1() => norms[1];
    
    
    /// <summary>
    /// Third normal (CCW)
    /// </summary>
    public readonly Vector3 N2() => norms[2];


    /// <summary>
    /// Alt normal accessor (CCW)
    /// </summary>
    public readonly Vector3 N( int index ) => norms[ index ];


    /// <summary>
    /// First vertex color (CCW)
    /// </summary>
    public readonly Vector4 C0() => color[0];
    

    /// <summary>
    /// Second vertex color (CCW)
    /// </summary>
    public readonly Vector4 C1() => color[1];
    
    
    /// <summary>
    /// Third vertex color (CCW)
    /// </summary>
    public readonly Vector4 C2() => color[2];


    /// <summary>
    /// Alt vertex color accessor (CCW)
    /// </summary>
    public readonly Vector4 C( int index ) => color[ index ];


    /// <summary>
    /// Convert a list of `Tri` as a plain array (for OGL)
    /// </summary>
    public static float[] Triangles2Arr( List<Tri> triangles ){
        float[] data = new float[ triangles.Count * 9 ];
        for (int i = 0; i < triangles.Count; i++){
            for( int j = 0; j < 3; ++j ){
                data[i * 9 + j * 3 + 0] = triangles[i][j][0];
                data[i * 9 + j * 3 + 1] = triangles[i][j][1];
                data[i * 9 + j * 3 + 2] = triangles[i][j][2];
            }
        }
        return data;
    }
}


////////// QUADS ///////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Quad
/// </summary>
public readonly struct Quad {

    /// Members ///
    private readonly Vector3[] /*----------*/ verts = new Vector3[4]; // Array ref cannot change, values can 
    private readonly Vector3[] /*----------*/ norms = new Vector3[4]; // Norm Directions 
    public readonly  Dictionary<string,float> attrs = []; // ----------- Dict ref cannot change, key-value pairs can
    

    /// <summary>
    /// Four points as a Quad
    /// </summary>
    public Quad( Vector3 a, Vector3 b, Vector3 c, Vector3 d ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
        verts[3] = d;
        Vector3 BA = a - b;
        Vector3 BC = c - b;
        Vector3 DC = c - d;
        Vector3 DA = a - d;
        SetNorms(  ((Vector3.Cross( BC, BA ) + Vector3.Cross( DA, DC ))/2f).Normalized()  );
    }


    /// <summary>
    /// Indexer: {get, set,}
    /// </summary>
    public readonly Vector3 this[int index]{
        get => verts[ index ];
        set => verts[ index ] = value;
    }


    /// <summary>
    /// First normal (CCW)
    /// </summary>
    public readonly Vector3 N0() => norms[0];
    

    /// <summary>
    /// Second normal (CCW)
    /// </summary>
    public readonly Vector3 N1() => norms[1];
    
    
    /// <summary>
    /// Third normal (CCW)
    /// </summary>
    public readonly Vector3 N2() => norms[2];

    
    /// <summary>
    /// Third normal (CCW)
    /// </summary>
    public readonly Vector3 N3() => norms[3];


    /// <summary>
    /// Alt normal accessor (CCW)
    /// </summary>
    public readonly Vector3 N( int index ) => norms[ index ];


    /// <summary>
    /// Alt normal accessor (CCW)
    /// </summary>
    public readonly Vector3 Navg() => (N0() + N1() + N2() + N3())/4f;


    /// <summary>
    /// Return a new `Quad` with the same coordinates and attributes
    /// </summary>
    public Quad Copy(){
        Quad qRtn = new( verts[0], verts[1], verts[2], verts[3] );
        foreach( (string k, float v) in attrs ){  qRtn.attrs[k] = v;  }
        return qRtn;
    }


    /// <summary>
    /// Set uniform norm for all vertices
    /// </summary>
    public void SetNorms( Vector3 norm ){
        norm.Normalize();
        norms[0] = norm;
        norms[1] = norm;
        norms[2] = norm;
        norms[3] = norm;
    }


    /// <summary>
    /// Set per vertex norms
    /// </summary>
    public void SetNorms( Vector3 a, Vector3 b, Vector3 c, Vector3 d ){
        norms[0] = a.Normalized();
        norms[1] = b.Normalized();
        norms[2] = c.Normalized();
        norms[3] = d.Normalized();
    }

    
    /// <summary>
    /// Express the Quad as 2 `Tri`s, Either left slash or right slash
    /// </summary>
    public Tri[] AsTris( bool left = true ){
        Tri[] rtnTri = new Tri[2];
        if( left ){
            rtnTri[0] = new Tri( verts[0], verts[1], verts[2] );
            rtnTri[1] = new Tri( verts[2], verts[3], verts[0] );
        }else{
            rtnTri[0] = new Tri( verts[1], verts[2], verts[3] );
            rtnTri[1] = new Tri( verts[3], verts[0], verts[1] );
        }
        return rtnTri;
    }


    /// <summary>
    /// Return a copy of the Quad mesh as a Tri Mesh
    /// </summary>
    public static List<Tri> AsTriMesh( IEnumerable<Quad> qMesh ){
        List<Tri> rtnMsh = [];
        rtnMsh.Capacity = qMesh.Count() * 2;
        foreach( Quad quad in qMesh ){  foreach( Tri tri in quad.AsTris() ){  rtnMsh.Add( tri );  }  }
        return rtnMsh;
    }


    /// <summary>
    /// First point (CCW)
    /// </summary>
    public readonly Vector3 V0() => verts[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector3 V1() => verts[1];
    
    
    /// <summary>
    /// Third point (CCW)
    /// </summary>
    public readonly Vector3 V2() => verts[2];

    /// <summary>
    /// Third point (CCW)
    /// </summary>
    public readonly Vector3 V3() => verts[3];


    /// <summary>
    /// Centroid of the Quad
    /// </summary>
    public Vector3 Center(){
        Vector3 rtnVec = new(0,0,0);
        foreach( Vector3 vec in verts ){  rtnVec += vec;  }
        return rtnVec / 4.0f;
    }

    /// <summary>
    /// Return a rotated a collection of `Quad`s
    /// </summary>
    public static List<Quad> RotateMesh( IEnumerable<Quad> mesh, Vector3 axis, float theta ){
        Quad quad_i;
        List<Quad> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count();
        axis.Normalize();
        Quaternion quat = MathVec3.AxisAngleQuat( axis, theta );
        foreach( Quad quad in mesh ){
            quad_i = quad.Copy();
            quad_i[0] = quat * quad.V0(); 
            quad_i[1] = quat * quad.V1(); 
            quad_i[2] = quat * quad.V2();
            quad_i[3] = quat * quad.V3(); 
            rtnMsh.Add( quad_i );
        }
        return rtnMsh;
    }


    /// <summary>
    /// Print the 4 CCW points on one line, followed by indented dictionary attributes,
    /// SLOP: https://claude.ai/share/a8e79d71-ba4c-434c-85b7-c2601bd2b4bf
    /// </summary>
    public override readonly string ToString(){
        StringBuilder sb = new();
        sb.Append( $"[{verts[0]}, {verts[1]}, {verts[2]}, {verts[3]}]" );
        if( attrs.Count > 0 ){
            foreach( string key in attrs.Keys.OrderBy( k => k, StringComparer.Ordinal ) ){
                sb.Append( $"\n\t{key}: {attrs[key]}" );
            }
        }
        return sb.ToString();
    }


    public static List<List<int>> GetFaces( List<Quad> qMesh ){
        List<List<int>> faces = [];
        List<int> /*-*/ face  = [];
        int /*-------*/ N     = qMesh.Count;
        Quad /*------*/ q_i, q_j;
        float /*-----*/ thresh = 2.5f / 180f * MathF.PI;
        HashSet<int>    used   = [];

        for( int i = 0; i < N-1; ++i ){
            if( !used.Contains(i) ){
                face.Add(i);
                used.Add(i);
            }else{  continue;  }
            q_i = qMesh[i];
            for( int j = i+1; j < N; ++j ){
                q_j = qMesh[j];
                if( MathVec3.AngleBetween( q_i.Navg(), q_j.Navg() ) <= thresh ){
                    face.Add(j);
                    used.Add(j);
                }
                
            }
            faces.Add( face );
            face = [];
        }
        return faces;
    }


    /// <summary>
    /// Return a the CCW edges of the `Quad` as a list of line segments
    /// </summary>
    public List<LinSeg> GetEdges(){
        List<LinSeg> edges =[];
        edges.Capacity = 4;
        edges.Add( new LinSeg( V0(), V1() ) );
        edges.Add( new LinSeg( V1(), V2() ) );
        edges.Add( new LinSeg( V2(), V3() ) );
        edges.Add( new LinSeg( V3(), V0() ) );
        return edges;
    }


    /// <summary>
    /// (Local) Int array comparator class,
    /// SLOP: https://claude.ai/share/b45d2337-0051-4268-a530-a3733ae491e2
    /// </summary>
    public class IntArrayComparer : IEqualityComparer<int[]> {
        
        /// <summary>
        /// Are two arrays equal?
        /// </summary>
        public bool Equals( int[]? x, int[]? y ){
            if (x is null || y is null) return x == y;
            return x.AsSpan().SequenceEqual(y);
        }

        /// <summary>
        /// Hashcode of the array
        /// </summary>
        public int GetHashCode( int[] arr ){
            var hash = new HashCode();
            foreach( var i in arr ){  hash.Add(i);  }
            return hash.ToHashCode();
        }
    }


    /// <summary>
    /// Return an ordered list of unshared line segments at the border of the `Quad` mesh,
    /// NOTE: Returned list may contain MULTIPLE LOOPS
    /// </summary>
    public static List<LinSeg> GetPerimeter( List<Quad> qMesh ){
        List<LinSeg>   perimeter = [];
        List<LinSeg>   ordered   = [];
        int /*------*/ N /*---*/ = qMesh.Count;
        Quad /*-----*/ q_i, q_j;
        List<LinSeg>   edges_i, edges_j;
        LinSeg /*---*/ edge_k , edge_l;
        HashSet<int[]> shared  = new( new IntArrayComparer() );
        HashSet<int>   used    = [];

        /// Phase 1: Find Unshared Edges ///

        for( int i = 0; i < N; ++i ){
            q_i     = qMesh[i];
            edges_i = q_i.GetEdges();
            for( int k = 0; k < 4; ++k ){
                if( shared.Contains([i,k]) ){  continue;  }
                edge_k = edges_i[k];
                for( int j = i+1; j < N; ++j ){
                    q_j     = qMesh[j];
                    edges_j = q_j.GetEdges();
                    for( int l = 0; l < 4; ++l ){
                        edge_l = edges_j[4];
                        if( edge_k.IsShared( edge_l ) ){
                            shared.Add([i,k]);
                            shared.Add([j,l]);
                        }
                    }
                }
                if( !shared.Contains([i,k]) ){  perimeter.Add( edges_i[k] );  }
            }
        }

        /// Phase 2: Order Unshared Edges ///
        ordered = [ perimeter[0], ];
        ordered.Capacity = perimeter.Count;
        LinSeg periSeg;
        int    count = 0;
        bool   added;
        bool   latch = false;

        while( (ordered.Count < perimeter.Count) && (count < (perimeter.Count*2)) ){
            added = false;
            for( int i = 1; i < perimeter.Count; ++i ){
                if( used.Contains(i) ){  continue;  }
                periSeg = perimeter[i];
                if( MathVec3.Equal( ordered[^1].V1(), periSeg.V0() ) ){
                    ordered.Add( periSeg );
                    used.Add(i);
                    added = true;
                }
            }
            // If we made two passes without adding, Just add next unadded
            if( (!added) && latch ){  
                for( int i = 1; i < perimeter.Count; ++i ){
                    if( used.Contains(i) ){  continue;  }
                    ordered.Add( perimeter[i] );
                    used.Add(i);
                    added = true;
                    break;
                }
            }
            latch = !added;
            ++count;
        }

        return ordered;
    }
}


}
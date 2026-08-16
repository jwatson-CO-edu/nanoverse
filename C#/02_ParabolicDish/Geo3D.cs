using OpenTK.Mathematics;


/// <summary>
/// Helper functions for 3D Geometry
/// </summary>
namespace geo3d {



////////// VECTOR3 MATH ////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Working with `OpenTK.Mathematics.Vector3`
/// </summary>
public class MathVec3 {

    const float _EPSILON = 0.00001f;

    
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
    public static Vector3 UniformPointCentroid( List<Vector3> points ){
        Vector3 rtnPnt = new(0,0,0);
        foreach( Vector3 pnt in points ){  rtnPnt += pnt;  }
        return rtnPnt / points.Count;
    }


    /// <summary>
    /// Return a shifted copy of the collection of points
    /// </summary>
    public static List<Vector3> ShiftPoints( List<Vector3> points, Vector3 shift ){
        List<Vector3> rtnLst = [];
        rtnLst.Capacity = points.Count;
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
    

}



////////////////////////////////////////////////////////////////////////////////////////////////////
////////// GEOMETRIC STRUCTS ///////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// TRIANGLES ///////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Triangle
/// </summary>
public readonly struct Tri {
    private readonly Vector3[] verts = new Vector3[3]; // Array ref cannot change, values can 

    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public Tri( Vector3 a, Vector3 b, Vector3 c ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
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
    public static float MeshArea( List<Tri> mesh ){
        float   totArea  = 0.0f;
        foreach( Tri tri in mesh ){  totArea += tri.Area();  }
        return totArea;
    }


    /// <summary>
    /// Return the area centroid of a collection of triangles
    /// </summary>
    public static Vector3 MeshAreaCentroid( List<Tri> mesh ){
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


    public static Vector3[] MeshBBox( List<Tri> mesh ){
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
    /// Shift a collection of triangles
    /// </summary>
    public static List<Tri> ShiftMesh( List<Tri> mesh, Vector3 shift ){
        Tri tri_i;
        List<Tri> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count;
        foreach( Tri tri in mesh ){
            tri_i = new( tri.V0() + shift, 
                         tri.V1() + shift, 
                         tri.V2() + shift );
            rtnMsh.Add( tri_i );
        }
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

    private readonly Vector3[] /*----------*/ verts = new Vector3[4]; // Array ref cannot change, values can 
    public readonly  Dictionary<string,float> attrs = []; // ----------- Dict ref cannot change, key-value pairs can
    
    /// <summary>
    /// Four points as a Quad
    /// </summary>
    public Quad( Vector3 a, Vector3 b, Vector3 c, Vector3 d ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
        verts[3] = d;
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
}



////////// POLYGON /////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// 3D N-gon, Flatness NOT guaranteed
/// </summary>
public class Polygon {
    public List<Vector3> /*------*/ verts = [];
    public List<Tri> /*----------*/ mesh  = [];
    public List<Quad> /*---------*/ qMesh = [];
    public Dictionary<string,float> attrs = []; // General data payload
}



}
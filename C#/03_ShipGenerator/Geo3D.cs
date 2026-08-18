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
    /// Generate a 3D vector with random {X,Y,Z,} components of a specified `scale`
    /// </summary>
    public static Vector3 NoiseXYZ( Random rand, float scale = 1.0f ){
        return new Vector3( rand.NextSingle()-0.5f, rand.NextSingle()-0.5f, rand.NextSingle()-0.5f ).Normalized() * scale;
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
        Vector3 ab     = V1() - V0();
        Vector3 ac     = V2() - V0();
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
    /// Alt vertex accessor (CCW)
    /// </summary>
    public readonly Vector3 V( int index ) => verts[ index ];


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
public readonly struct Quad{
    private readonly Vector3[] verts = new Vector3[4]; // Array ref cannot change, values can 

    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public Quad( Vector3 a, Vector3 b, Vector3 c, Vector3 d ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
        verts[3] = d;
    }


    /// <summary>
    /// Centroid of the Quad
    /// </summary>
    public Vector3 Center(){
        Vector3 rtnVec = new(0,0,0);
        foreach( Vector3 vec in verts ){  rtnVec += vec;  }
        return rtnVec / 4.0f;
    }
    
}

}
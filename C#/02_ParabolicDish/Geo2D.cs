using OpenTK.Mathematics;
using geo3d;




/// <summary>
/// Helper functions for 2D Geometry
/// </summary>
namespace geo2d {



////////// VECTOR3 MATH ////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Working with `OpenTK.Mathematics.Vector3`
/// </summary>
public static class MathVec2 {

    const float _EPSILON = 0.00001f;

    
    /// <summary>
    /// Are the vectors equal?
    /// </summary>
    public static bool Equal( Vector2 vec1 , Vector2 vec2 ){  return (vec1 - vec2).Length < _EPSILON;  }

    public static List<Segment> ProjectSegmentsDown( List<LinSeg> segments3d ){
        List<Segment> rtnLst = [];
        List<Vector3> segPts = [];
        segPts.Capacity = segments3d.Count;
        rtnLst.Capacity = segments3d.Count;

        foreach( LinSeg lSeg in segments3d ){  segPts.Add( lSeg.V0() );  }

        /// Phase 1: Project Onto Plane ///
        List<Vector2> flatList = Ops2D3D.Project3dPointsTo2d( segPts );
        
        /// Phase 2: Create segments ///
        Vector2 lst = flatList[^1];
        foreach( Vector2 pnt in flatList ){
            rtnLst.Add( new Segment( pnt, lst ) );
            lst = pnt;
        }

        return rtnLst;
    }

}


////////////////////////////////////////////////////////////////////////////////////////////////////
////////// GEOMETRIC STRUCTS ///////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


////////// SEGMENTS ///////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Segment
/// </summary>
public readonly struct Segment{

    /// Members ///
    // Array refs cannot change, values can
    private readonly Vector2[] /*----------*/ verts = new Vector2[2]; // Vertex Positions 
    private readonly Vector4[] /*----------*/ color = new Vector4[2]; // Vertex Colors 
    public  readonly Dictionary<string,float> attrs = []; // ----------- Attributes
    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public Segment( Vector2 a, Vector2 b ){
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
    public readonly Vector2 V0() => verts[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector2 V1() => verts[1];


    /// <summary>
    /// First point (CCW)
    /// </summary>
    public readonly Vector4 C0() => color[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector4 C1() => color[1];


    /// <summary>
    /// Centroid of a collection of `Segment`s
    /// </summary>
    public static Vector2 Centroid( IEnumerable<Segment> cllctn ){
        Vector2 center = new(0,0);
        foreach( Segment seg in cllctn ){  center += seg.V0() + seg.V1();  }
        return center / (cllctn.Count() * 2);
    }


    /// <summary>
    /// Calculate the collection Axis-Aligned Bounding Box (AABB)
    /// </summary>
    public static Vector2[] BBox( IEnumerable<Segment> cllctn ){
        Vector2 lo = new(  6e10f,  6e10f );
        Vector2 hi = new( -6e10f, -6e10f );
        foreach( Segment seg in cllctn ){
            foreach( Vector2 pt in seg.verts ){
                for( int k = 0; k < 2; ++k ){
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
    public static List<Segment> CopySegments( IEnumerable<Segment> cllctn ){
        List<Segment> rtnLst = [];
        Segment nuSeg;
        rtnLst.Capacity = cllctn.Count();
        foreach( Segment seg in cllctn ){
            nuSeg = new Segment( seg.V0(), seg.V1() );
            nuSeg.color[0] = new Vector4( seg.C0() );
            nuSeg.color[1] = new Vector4( seg.C1() );
            foreach( (string k, float v) in seg.attrs ){  nuSeg.attrs[k] = v;  }
            rtnLst.Add( nuSeg );
        }
        return rtnLst;
    }


    /// <summary>
    /// Return a shifted a collection of `Segment`s
    /// </summary>
    public static List<Segment> ShiftSegments( IEnumerable<Segment> cllctn, Vector2 shift ){
        List<Segment> rtnLst = CopySegments( cllctn );
        for( int i = 0; i < rtnLst.Count; ++i ){
            rtnLst[i].verts[0] += shift; 
            rtnLst[i].verts[1] += shift; 
        }
        return rtnLst;
    }


    /// <summary>
    /// Set a uniform weight for collection of `Segment`s
    /// </summary>
    public static void SetWeight( IEnumerable<Segment> cllctn, float weight ){
        foreach( Segment seg in cllctn ){  seg.attrs["weight"] = weight;  }
    }


    /// <summary>
    /// Return a rotated a collection of `Segment`s
    /// </summary>
    public static List<Segment> RotateSegments( IEnumerable<Segment> segments, Vector2 center, float theta ){
        List<Segment> rtnLst = ShiftSegments( segments, -center );
        Matrix2 /*-*/ matx   = Matrix2.CreateRotation( theta );
        for( int i = 0; i < rtnLst.Count; ++i ){
            rtnLst[i].verts[0] = matx * rtnLst[i].verts[0]; 
            rtnLst[i].verts[1] = matx * rtnLst[i].verts[1]; 
        }
        return ShiftSegments( rtnLst, center );
    }

}



////////// TRIANGLES ///////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Triangle
/// </summary>
public readonly struct Triangle{

    /// Members ///
    // Array refs cannot change, values can
    private readonly Vector2[] verts = new Vector2[3]; // Vertex Positions 
    private readonly Vector4[] color = new Vector4[3]; // Vertex Colors 

    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public Triangle( Vector2 a, Vector2 b, Vector2 c ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
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
    public void SetColor( Vector3 a, Vector3 b, Vector3 c, float alpha = 1f ){
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
    public Vector2 Center(){
        Vector2 rtnVec = new(0,0);
        foreach( Vector2 vec in verts ){  rtnVec += vec;  }
        return rtnVec / 3.0f;
    }


    /// <summary>
    /// Area of the Triangle, Heron's Formula
    /// </summary>
    public float Area(){
        float a = (V1() - V0()).Length;
        float b = (V2() - V1()).Length;
        float c = (V0() - V2()).Length;
        float s = (a+b+c)/2f;
        return MathF.Sqrt( s * (s-a) * (s-b) * (s-c) );
    }


    /// <summary>
    /// Return the area centroid of a collection of triangles
    /// </summary>
    public static float MeshArea( IEnumerable<Triangle> mesh ){
        float   totArea  = 0.0f;
        foreach( Triangle tri in mesh ){  totArea += tri.Area();  }
        return totArea;
    }


    /// <summary>
    /// Return the area centroid of a collection of triangles
    /// </summary>
    public static Vector2 MeshAreaCentroid( IEnumerable<Triangle> mesh ){
        float   area_i;
        float   totArea  = 0.0f;
        Vector2 centroid = new(0,0);
        foreach( Triangle tri in mesh ){
            area_i    = tri.Area();
            totArea  += area_i;
            centroid += tri.Center() * area_i;
        }
        return centroid / totArea;
    }


    /// <summary>
    /// Calculate the mesh Axis-Aligned Bounding Box (AABB)
    /// </summary>
    public static Vector2[] MeshBBox( IEnumerable<Triangle> mesh ){
        Vector2 lo = new(  6e10f,  6e10f );
        Vector2 hi = new( -6e10f, -6e10f );
        foreach( Triangle tri in mesh ){
            foreach( Vector2 pt in tri.verts ){
                for( int k = 0; k < 2; ++k ){
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
    public static List<Triangle> ShiftMesh( IEnumerable<Triangle> mesh, Vector2 shift ){
        Triangle tri_i;
        List<Triangle> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count();
        foreach( Triangle tri in mesh ){
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
    public static List<Triangle> RotateMesh( IEnumerable<Triangle> mesh, Vector2 center, float theta ){
        List<Triangle> rtnMsh = ShiftMesh( mesh, -center );
        Matrix2 /*--*/ matx   = Matrix2.CreateRotation( theta );
        for( int i = 0; i < rtnMsh.Count; ++i ){
            rtnMsh[i] = new( matx * rtnMsh[i].V0(), 
                             matx * rtnMsh[i].V1(), 
                             matx * rtnMsh[i].V2() );
        }
        return ShiftMesh( rtnMsh, center );
    }


    /// <summary>
    /// Copy the entire mesh
    /// </summary>
    public static List<Triangle> CopyMesh( IEnumerable<Triangle> mesh ){
        List<Triangle> rtnMsh = [];
        rtnMsh.Capacity = mesh.Count();
        foreach( Triangle tri in mesh ){
            rtnMsh.Add( new Triangle( tri.V0(), 
                                      tri.V1(), 
                                      tri.V2() ) );
        }
        return rtnMsh;
    }


    /// <summary>
    /// Apply one color to the entire mesh
    /// </summary>
    public static List<Triangle> ColorMesh( IEnumerable<Triangle> mesh, Vector4 color ){
        List<Triangle> rtnMsh = CopyMesh( mesh );
        foreach( Triangle tri in rtnMsh ){  tri.SetColor( color );  }
        return rtnMsh;
    }


    /// <summary>
    /// Indexer: {get, set,}
    /// </summary>
    public readonly Vector2 this[int index]{
        get => verts[ index ];
        set => verts[ index ] = value;
    }


    /// <summary>
    /// First point (CCW)
    /// </summary>
    public readonly Vector2 V0() => verts[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector2 V1() => verts[1];
    
    
    /// <summary>
    /// Third point (CCW)
    /// </summary>
    public readonly Vector2 V2() => verts[2];


    /// <summary>
    /// Alt vertex accessor (CCW)
    /// </summary>
    public readonly Vector2 V( int index ) => verts[ index ];


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
    public static float[] Triangles2Arr( List<Triangle> triangles ){
        float[] data = new float[ triangles.Count * 6 ];
        for (int i = 0; i < triangles.Count; i++){
            for( int j = 0; j < 3; ++j ){
                data[i * 6 + j * 2 + 0] = triangles[i][j][0];
                data[i * 6 + j * 2 + 1] = triangles[i][j][1];
                data[i * 6 + j * 2 + 2] = triangles[i][j][2];
            }
        }
        return data;
    }
}

/* ///////// DEV_PLAN //////////////////////////////////////////////////////////////////////////////
* 2026-08-25 : NO LAYERS at this time (YAGNI)
[ ] 
*/


/// <summary>
/// 2D Figure composed of 2D `Segment`s
/// </summary>
public class Figure {
    public List<Segment> segments = [];


    public void AddSegments( IEnumerable<Segment> nuSegments ){  segments.AddRange( nuSegments );  }


    public static List<Segment> MakeTrapezoid( float height, float topLength, float bottomLength ){
        List<Segment> rtnLst = [];
        rtnLst.Capacity = 4;
        float hlfHgt = height / 2f;
        float hlfTop = topLength / 2f;
        float hlfBtm = bottomLength / 2f;
        rtnLst.Add( new Segment( new Vector2(  hlfTop,  hlfHgt ), new Vector2( -hlfTop,  hlfHgt ) ) );
        rtnLst.Add( new Segment( new Vector2( -hlfTop,  hlfHgt ), new Vector2( -hlfBtm, -hlfHgt ) ) );
        rtnLst.Add( new Segment( new Vector2( -hlfBtm, -hlfHgt ), new Vector2(  hlfBtm, -hlfHgt ) ) );
        rtnLst.Add( new Segment( new Vector2(  hlfBtm, -hlfHgt ), new Vector2(  hlfTop,  hlfHgt ) ) );
        return rtnLst;
    }
}

}



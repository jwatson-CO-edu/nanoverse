using OpenTK.Mathematics;
using curve;

namespace sigil {


/// <summary>
/// Triangle
/// </summary>
public readonly struct Tri{
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



/// <summary>
/// A curve with thickness in 3D space, NOTE: At this time strokes have uniform thickness
/// </summary>
public class Stroke {
    
    public const int   _DEFAULT_DIV = 64;
    public const float _UNDER_STEP  = 1f/128f; 
    public float /*-*/ borderMult   = 1.125f;
    public float /*-*/ thick;
    public Parametric  curve;
    public int /*---*/ div;
    public float /*-*/ dt;
    public List<Tri>   geo;
    public List<Tri>   brdr;
    public List<float> tGeo;
    public List<float> zOffset;
    public List<float> zBorder;
    

    // <summary>
    /// Create a bi-directional edge between `this` and `other`
    /// </summary>
    public bool HasNeighbor( Stroke other ){  return curve.HasNeighbor( other.curve );  }
    
    
    /// <summary>
    /// Is `this` connected to `other`?
    /// </summary>
    public void ConnectBidir( Stroke other ){  curve.ConnectBidir( other.curve );  }

    
    /// <summary>
    /// Default constructor
    /// </summary>
    public Stroke(){
        thick   = 0.0f;
        curve   = new DummyCurve();
        div     = _DEFAULT_DIV;
        geo     = [];
        brdr    = [];
        tGeo    = [];
        zOffset = [];
        zBorder = [];
    }


    /// <summary>
    /// Return true only if an underlying parametric curve was set
    /// </summary>
    public bool HasCurve(){
        if( curve is DummyCurve ){  return false;  }
        return true;
    }

    
    /// <summary>
    /// Set curve and thickness
    /// </summary>
    public Stroke( Parametric param, float thickness, int div_ = _DEFAULT_DIV ){
        thick   = thickness;
        curve   = param;
        div     = div_;
        geo     = [];
        brdr    = [];
        tGeo    = [];
        zOffset = [];
        zBorder = [];
    }


    /// <summary>
    /// Reserve space for mesh and init the Z-offset array
    /// </summary>
    public void ReserveGeo(){
        int NendPt /*-*/ = div+1;
        geo.Capacity     = 2*div;
        tGeo.Capacity    = NendPt;
        zOffset.Capacity = NendPt;
        dt /*---------*/ = 1.0f/div;
        for( int i = 0; i < NendPt; ++i ){
            tGeo.Add( i * dt );
            zOffset.Add(0f);
            zBorder.Add( -_UNDER_STEP );
        }
    }


    /// <summary>
    /// Create mesh for drawing. WARNING: This function req's that backface culling is >>OFF<<
    /// </summary>
    public void BuildGeo(){
        float t;
        float hlf = thick / 2.0f;
        Vector3 pt0, pt1, pt2, pt3, mid1, mid2;
        // tGeo.Add(0f);

        Console.WriteLine( $"There are {tGeo.Count} points to evaluate ..." );

        for( int i = 0; i < tGeo.Count; i++ ){ 

            t   = tGeo[i];
            Console.WriteLine( $"\tt = {t}" );

            mid1 = curve.Val( t    );
            mid2 = curve.Val( t+dt );
            
            pt0 = mid1 + curve.Crv( t    ).Normalized() * hlf;
            pt1 = mid1 - curve.Crv( t    ).Normalized() * hlf;
            pt2 = mid2 + curve.Crv( t+dt ).Normalized() * hlf;
            pt3 = mid2 - curve.Crv( t+dt ).Normalized() * hlf;

            geo.Add( new Tri( pt0, pt1, pt3 ) );
            geo.Add( new Tri( pt2, pt3, pt0 ) );
        }

        Console.WriteLine( $"Evaluated {tGeo.Count} ..." );
    }


    /// <summary>
    /// Return all the segments of each curve within specified distance of the other 
    /// </summary>
    public List<List<float>> GetIntersections( Stroke other, float dt, float margin = 0f ){
        List<List<float>> rtnLst = [[],[]];
        float /*-------*/ d_ij;
        Vector3 /*-----*/ vThis;
        Vector3 /*-----*/ vOthr;
        bool /*--------*/ pairOpen = false;
        if( margin < dt ){  margin = thick/3f;  }
        float thresh = (thick + other.thick)/2f + margin;

        foreach( float tThis in tGeo ){
            vThis = curve.Val( tThis );
            foreach( float tOthr in other.tGeo ){
                vOthr = other.curve.Val( tOthr );
                d_ij  = Vector3.Distance( vThis, vOthr );
                if( !pairOpen ){
                    if( d_ij <= thresh ){
                        pairOpen = true;
                        rtnLst[0].Add( tThis );
                        rtnLst[1].Add( tOthr );
                    }
                }else{
                    if( d_ij > thresh ){
                        pairOpen = false;
                        rtnLst[0].Add( tThis );
                        rtnLst[1].Add( tOthr );
                    }
                }
            }
        }
        return rtnLst;
    }


}





}
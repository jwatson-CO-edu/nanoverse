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
}



/// <summary>
/// A curve with thickness in 3D space, NOTE: At this time strokes have uniform thickness
/// </summary>
public class Stroke {
    private static int /*--*/ nextID /*-*/ = 0;
    public int /*----------*/ id /*-----*/ = 0;
    public const int /*----*/ _DEFAULT_DIV = 32;
    public float /*--------*/ thick;
    public Parametric /*---*/ curve;
    public int /*----------*/ div;
    public List<Tri> /*----*/ geo;
    public List<float> /*--*/ tGeo;
    public LinkedList<Stroke> edges;


    public static int NextID(){
        int rtn = nextID;
        nextID++;
        return rtn;
    }

    
    /// <summary>
    /// Default constructor
    /// </summary>
    public Stroke(){
        id    = NextID();
        thick = 0.0f;
        curve = new DummyCurve();
        div   = _DEFAULT_DIV;
        geo   = [];
        tGeo  = [];
        edges = [];
    }

    
    /// <summary>
    /// Set curve and thickness
    /// </summary>
    public Stroke( Parametric param, float thickness, int div_ = _DEFAULT_DIV ){
        id    = NextID();
        thick = thickness;
        curve = param;
        div   = div_;
        geo   = [];
        tGeo  = [];
        edges = [];
        geo.Capacity  = 2*div;
        tGeo.Capacity = div+1;
    }


    /// <summary>
    /// Does `nghbrID` represent an edge for this `Stroke`
    /// </summary>
    public bool IsNeighbor( int nghbrID ){
        foreach( Stroke neighbor in edges ){  if( neighbor.id == nghbrID ){  return true;  }  }
        return false;
    }


    /// <summary>
    /// Create a bi-directional edge between `this` and `other`
    /// </summary>
    public void ConnectBidir( Stroke other ){
        edges.AddLast( other );
        other.edges.AddLast( this );
    }


    /// <summary>
    /// Create mesh for drawing. WARNING: This function req's that backface culling is >>OFF<<
    /// </summary>
    public void BuildGeo(){
        float t   = 0.0f;
        float dt  = 1.0f/div;
        float hlf = thick / 2.0f;
        Vector3 pt0, pt1, pt2, pt3, mid;
        tGeo.Add(0f);
        while( t < 1.0f ){ 
            mid = curve.Val(t);
            
            pt0 = mid + curve.Crv( t    ).Normalized() * hlf;
            pt1 = mid - curve.Crv( t    ).Normalized() * hlf;
            pt2 = mid + curve.Crv( t+dt ).Normalized() * hlf;
            pt3 = mid - curve.Crv( t+dt ).Normalized() * hlf;

            geo.Add( new Tri( pt0, pt1, pt3 ) );
            geo.Add( new Tri( pt0, pt3, pt2 ) );

            t += dt;
            tGeo.Add(t);
        }
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
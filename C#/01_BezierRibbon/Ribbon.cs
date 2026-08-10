using OpenTK.Mathematics;
using parametric;
using geo3d;

namespace ribbon {

public class Constants {
    public const int   _DEFAULT_DIV = 64;
    public const float _LAYER_SEP   = 1f / 64f;
}

/// <summary>
/// Flat 
/// </summary>
public class Ribbon {
    
    public int /*-----*/ div; // -- Number of chunks that define the ribbon
    public int /*-----*/ N; // ---- Number of boundaries that define the ribbon
    public float /*---*/ twist; //- Radians of twist along `t` = [0,1]
    public float /*---*/ dTheta; // Twist along each chunk  
    public Parametric    spine; //- Curve at the center of the ribbon
    public float /*---*/ dt; // --- Parametric length of each chunk
    public float /*---*/ width; //- Uniform width of the ribbon
    public float /*---*/ margin; //- Uniform width of the ribbon
    public List<float>   tLst; // - Parameter of chunk borders
    public List<float>   Xwdt; // - Width along X at chunk borders
    public List<Vector3> Xdir; // - Width direction at chunk borders
    public List<Tri>     top; // -- Top ribbon
    public List<Tri>     mid; // -- Middle border
    public List<Tri>     btm; // -- Bottom ribbon
    
    
    /// <summary>
    /// Default Constructor 
    /// </summary>
    public Ribbon ( float width_, int div_ = Constants._DEFAULT_DIV, float twist_ = 0f ){
        width  = width_;
        margin = 1.125f;
        div    = div_;
        N /**/ = div_+1;
        twist  = twist_;
        dt     = 1.0f / div;
        dTheta = twist / div;
        spine  = new DummyCurve();
        tLst   = [];
        Xwdt   = [];
        Xdir   = [];
        top    = [];
        mid    = [];
        btm    = [];
        // Reserve Geo
        Xdir.Capacity = N;
        tLst.Capacity = N;
        Xwdt.Capacity = N;
        top.Capacity  = 2*div;
        mid.Capacity  = 2*div;
        btm.Capacity  = 2*div;        
        for( int i = 0; i <= div; ++i ){  tLst.Add( i * dt ); }
    }


    /// <summary>
    /// Get the index of the segment on which `t` lies 
    /// </summary>
    public int GetSegment( float t ){
        for( int i = 0; i < N; ++i ){  if( tLst[i] > t ){  return i-1;  }  }
        return N-1;
    }


    // FIXME: NEEDS TESTING
    /// <summary>
    /// Set the X-direction at a certain `t` and back out the X-direction of surrounding segment boundaries 
    /// </summary>
    public void SetXdirAt( Vector3 xDir, float t = 0f ){
        int     i = GetSegment(t);
        float   turn;
        Vector3 Zdir, Ydir;
        Xdir[i] = xDir;
        for( int ii = i-1; ii < N; ++ii ){
            turn = (tLst[ii] - t)*dTheta;
            Zdir = spine.Tan(t).Normalized();
            Ydir = Vector3.Cross( Zdir, xDir ).Normalized();
            xDir = Vector3.Cross( Ydir, Zdir ).Normalized();
            Xdir[ ii ] = xDir;
            xDir = MathVec3.AxisAngleQuat( Zdir, turn ) * xDir;
            t    = tLst[ii];
        }
        if( i > 0 ){
            for( int ii = i-1; ii > -1; --ii ){
                turn = (tLst[ii] - t)*dTheta;
                Zdir = spine.Tan(t).Normalized();
                Ydir = Vector3.Cross( Zdir, xDir ).Normalized();
                xDir = Vector3.Cross( Ydir, Zdir ).Normalized();
                Xdir[ ii ] = xDir;
                xDir = MathVec3.AxisAngleQuat( Zdir, turn ) * xDir;
                t    = tLst[ii];
            }
        }
    }


    /// <summary>
    /// Create mesh for drawing. WARNING: This function req's that backface culling is >>OFF<<
    /// </summary>
    public void BuildGeo(){
        float t;
        float hlf = width / 2.0f;
        Vector3 Zdir, Ydir, pt0, pt1, pt2, pt3, mid1, mid2;
        for( int i = 0; i < N-1; ++i ){
            t    = tLst[i];
            Zdir = spine.Tan(t).Normalized();
            Ydir = Vector3.Cross( Zdir, Xdir[i] ).Normalized();

            mid1 = spine.Val( t    );
            mid2 = spine.Val( t+dt );

            /// Middle (Border) Geometry ///
            pt0 = mid1 + Xdir[i  ].Normalized() * hlf * margin;
            pt1 = mid1 - Xdir[i  ].Normalized() * hlf * margin;
            pt2 = mid2 + Xdir[i+1].Normalized() * hlf * margin;
            pt3 = mid2 - Xdir[i+1].Normalized() * hlf * margin;

            mid.Add( new Tri( pt0, pt1, pt3 ) );
            mid.Add( new Tri( pt2, pt3, pt0 ) );

            /// Top (Visual) Geometry ///
            pt0 = mid1 + Xdir[i  ].Normalized() * hlf + Ydir * Constants._LAYER_SEP;
            pt1 = mid1 - Xdir[i  ].Normalized() * hlf + Ydir * Constants._LAYER_SEP;
            pt2 = mid2 + Xdir[i+1].Normalized() * hlf + Ydir * Constants._LAYER_SEP;
            pt3 = mid2 - Xdir[i+1].Normalized() * hlf + Ydir * Constants._LAYER_SEP;

            top.Add( new Tri( pt0, pt1, pt3 ) );
            top.Add( new Tri( pt2, pt3, pt0 ) );

            /// Bottom (Visual) Geometry ///
            pt0 = mid1 + Xdir[i  ].Normalized() * hlf - Ydir * Constants._LAYER_SEP;
            pt1 = mid1 - Xdir[i  ].Normalized() * hlf - Ydir * Constants._LAYER_SEP;
            pt2 = mid2 + Xdir[i+1].Normalized() * hlf - Ydir * Constants._LAYER_SEP;
            pt3 = mid2 - Xdir[i+1].Normalized() * hlf - Ydir * Constants._LAYER_SEP;

            btm.Add( new Tri( pt0, pt1, pt3 ) );
            btm.Add( new Tri( pt2, pt3, pt0 ) );
        }
    }
    

}
    
}
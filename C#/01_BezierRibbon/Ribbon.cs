using OpenTK.Mathematics;
using parametric;
using geo3d;
using pose3d;

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
    public List<float>   tLst; // - Parameter of chunk borders
    public List<float>   Xwdt; // - Width along X at chunk borders
    public List<Matrix4> matx; // - Width direction at chunk borders
    public List<Tri>     top; // -- Top ribbon
    public List<Tri>     mid; // -- Middle border
    public List<Tri>     btm; // -- Bottom ribbon
    public bool /*----*/ hasMid; // Flag: Should the middle layer be constructed?


    /*     Y  +X ====================
           | /
           +--- Z    --RIBBON->
         /
       -X ====================     */
    
    
    /// <summary>
    /// Default Constructor 
    /// </summary>
    public Ribbon ( int div_ = Constants._DEFAULT_DIV, float twist_ = 0f, bool makeMid = true ){
        // Setup Params
        div    = div_;
        N /**/ = div_+1;
        twist  = twist_;
        dt     = 1.0f / div;
        dTheta = twist / div;
        spine  = new DummyCurve();
        tLst   = [];
        Xwdt   = [];
        matx   = [];
        top    = [];
        mid    = [];
        btm    = [];
        hasMid = makeMid;
        // Reserve Geo
        matx.Capacity = N;
        tLst.Capacity = N;
        Xwdt.Capacity = N;
        top.Capacity  = 2*div;
        mid.Capacity  = 4*div;
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


    /// <summary>
    /// Set the twist across the curve 
    /// </summary>
    public void SetTwist( float twist_ = 0f ){
        twist  = twist_;
        dTheta = twist / div;
    }


    /// <summary>
    /// Set the `Ribbon` orientation at t=0 
    /// </summary>
    public void SetXdirAt0( Vector3 xDir, bool compute = true ){
        Vector3 zDir;
        matx.Clear();
        matx.Capacity = N;
        zDir = spine.Tan(0).Normalized();
        matx.Add( MathMatx4.HomogFromXZBases( xDir, zDir, spine.Val(0) ) );
        if( compute ){  ComputeMatrices();  }
    }


    /// <summary>
    /// Set the `Ribbon` orientation at t>0, ASSUMPTION: `SetXdirAt0()` has already been called!
    /// </summary>
    public void ComputeMatrices(){
        Vector3 Z_im1, X_im1, Z_i, X_i;
        for( int i = 1; i < N; ++i ){
            X_im1 = MathMatx4.GetXBasis( matx[i-1] );
            Z_im1 = MathMatx4.GetZBasis( matx[i-1] );
            X_i   = MathVec3.AxisAngleQuat( Z_im1, dTheta ) * X_im1;
            Z_i   = spine.Tan( tLst[i] );
            matx.Add( MathMatx4.HomogFromXZBases( X_i, Z_i, spine.Val( tLst[i] ) ) );
        }
    }



    /// <summary>
    /// Create mesh for drawing. WARNING: This function req's that backface culling is >>OFF<<
    /// </summary>
    public void BuildGeo( float width1, float width2 = 0f ){
        float t_i, t_ip1, w_i, w_ip1, h_i, h_ip1;
        if( width2 < 0.00001 ){  width2 = width1;  }
        
        float dW /**/ = (width2 - width1) / div;
        float margin  = Math.Max( width1, width2 ) / 8f;
        float[] width = new float[N];

        for( int i = 0; i < N; ++i ){  width[i] = width1 + i * dW;  }
        
        Vector3 X_i, X_ip1, Y_i, mid_i, mid_ip1, pt0, pt1, pt2, pt3;
        
        for( int i = 0; i < N-1; ++i ){
            t_i     = tLst[i];
            t_ip1   = tLst[i+1];
            w_i     = width[i];
            w_ip1   = width[i+1];
            h_i     = w_i/2f;
            h_ip1   = w_ip1/2f;
            X_i     = MathMatx4.GetXBasis( matx[i] );
            X_ip1   = MathMatx4.GetXBasis( matx[i+1] );
            Y_i     = MathMatx4.GetYBasis( matx[i] );
            mid_i   = spine.Val( t_i   );
            mid_ip1 = spine.Val( t_ip1 );

            /// Middle (Border) Geometry ///
            pt0 = mid_i   + X_i   * (h_i   + margin);
            pt1 = mid_i   - X_i   * (h_i   + margin);
            pt2 = mid_ip1 + X_ip1 * (h_ip1 + margin);
            pt3 = mid_ip1 - X_ip1 * (h_ip1 + margin);

            // Top Middle // 
            mid.Add( new Tri( pt0, pt1, pt3 ) );
            mid.Add( new Tri( pt2, pt3, pt1 ) );
            // Bottom Middle //
            mid.Add( new Tri( pt0, pt3, pt1 ) );
            mid.Add( new Tri( pt2, pt1, pt3 ) );

            // FIXME: START HERE - USE THE ABOVE CONVENTION

            /// Top (Visual) Geometry ///
            pt0 = mid1 + Xdir[i  ].Normalized() * hlf + Ydir * Constants._LAYER_SEP;
            pt1 = mid1 - Xdir[i  ].Normalized() * hlf + Ydir * Constants._LAYER_SEP;
            pt2 = mid2 + Xdir[i+1].Normalized() * hlf + Ydir * Constants._LAYER_SEP;
            pt3 = mid2 - Xdir[i+1].Normalized() * hlf + Ydir * Constants._LAYER_SEP;

            top.Add( new Tri( pt0, pt1, pt3 ) ); // FIXME: USE ABOVE ORDER
            top.Add( new Tri( pt2, pt3, pt0 ) );

            /// Bottom (Visual) Geometry ///
            pt0 = mid1 + Xdir[i  ].Normalized() * hlf - Ydir * Constants._LAYER_SEP;
            pt1 = mid1 - Xdir[i  ].Normalized() * hlf - Ydir * Constants._LAYER_SEP;
            pt2 = mid2 + Xdir[i+1].Normalized() * hlf - Ydir * Constants._LAYER_SEP;
            pt3 = mid2 - Xdir[i+1].Normalized() * hlf - Ydir * Constants._LAYER_SEP;

            btm.Add( new Tri( pt0, pt1, pt3 ) ); // FIXME: USE ABOVE ORDER
            btm.Add( new Tri( pt2, pt3, pt0 ) );
        }
    }
    

}
    
}
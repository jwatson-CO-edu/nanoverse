using OpenTK.Mathematics;
using parametric;
using geo3d;

namespace ribbon {

public class Constants {
    public const int _DEFAULT_DIV = 64;
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
    public List<Vector3> Xdir; // - Width direction at chunk borders
    public List<Tri>     top; // -- Top ribbon
    public List<Tri>     mid; // -- Middle border
    public List<Tri>     btm; // -- Bottom ribbon
    
    
    /// <summary>
    /// Default Constructor 
    /// </summary>
    public Ribbon ( int div_ = Constants._DEFAULT_DIV, float twist_ = 0f ){
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


    /// <summary>
    /// Set the X-direction at a certain `t` and back out the X-direction of surrounding segment boundaries 
    /// </summary>
    public void SetXdirAt( Vector3 xDir, float t = 0f ){
        int     i    = GetSegment(t);
        Vector3 Zdir = spine.Tan(t).Normalized();
        Vector3 Ydir = Vector3.Cross( Zdir, xDir ).Normalized();
        if( i > 0 ){

        }
    }
    

}
    
}
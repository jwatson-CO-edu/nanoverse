using OpenTK.Mathematics;
using parametric;

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
    public List<float>   tLst; // ---- Parameter of chunk borders
    public List<float>   Xwdt; // - Width along X at chunk borders
    public List<Vector3> Xdir; // - Width direction at chunk borders
    
    
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
        Xdir.Capacity = N;
        tLst.Capacity    = N;
        Xwdt.Capacity = N;
        for( int i = 0; i <= div; ++i ){  tLst.Add( i * dt ); }
    }


    public int GetSegment( float t ){
        for( int i = 0; i < N; ++i ){  if( tLst[i] > t ){  return i-1;  }  }
        return N-1;
    }


    public void SetXdirAt( Vector3 xDir, float t = 0f ){
        int     i    = GetSegment(t);
        Vector3 Zdir = spine.Tan(t).Normalized();
        Vector3 Ydir = Vector3.Cross( Zdir, xDir ).Normalized();
        if( i > 0 ){

        }
    }
    

}
    
}
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
    
    public int /*-----*/ div; // - Number of chunks that define the ribbon
    public float /*---*/ twist; // Radians of twist along `t` = [0,1]  
    public Parametric    spine; // Curve at the center of the ribbon
    public float /*---*/ dt; // -- Parametric length of each chunk
    public List<float>   t; // --- Parameter of chunk borders
    public List<float>   Xwdt; //- Width along X at chunk borders
    public List<Vector3> Xdir; //- Width direction at chunk borders
    
    
    public Ribbon ( int div_ = Constants._DEFAULT_DIV, float twist_ = 0f ){
        div   = div_;
        twist = twist_;
        dt    = 1.0f / div;
        spine = new DummyCurve();
        t     = [];
        Xwdt  = [];
        Xdir  = [];
        Xdir.Capacity = div_+1;
        t.Capacity    = div_+1;
        Xwdt.Capacity = div_+1;
        for( int i = 0; i <= div; ++i ){  t.Add( i * dt ); }
    }
    

}
    
}
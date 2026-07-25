using OpenTK.Mathematics;

namespace spline {

/// <summary>
/// Abstract base class for all Parametric curves 
/// </summary>
public abstract class Parametric {
    public abstract Vector3 Val( float t );
}


/// <summary>
/// Bezier Curves (Component function of a B-Spline)
/// </summary>
public class Bezier {


    /// <summary>
    /// Quadratic Bezier Curve
    /// </summary>
    public class Quad ( Vector3 P0, Vector3 P1, Vector3 P2 ) : Parametric {

        public static Vector3 Value( Vector3 P0, Vector3 P1, Vector3 P2, float t  ){
            float oneMinusT = 1.0f - t;
            float omtSquard = oneMinusT * oneMinusT;
            float tSquared  = t * t;
            return omtSquard*P0 + 2*oneMinusT*t*P1 + tSquared*P2;
        }

        public Vector3 p0 = P0;
        public Vector3 p1 = P1;
        public Vector3 p2 = P2; 

        public override Vector3 Val( float t ){  return Value( p0, p1, p2, t );  }

    }


    /// <summary>
    /// Cubic Bezier Curve
    /// </summary>
    public class Cubic ( Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3 ) : Parametric {

        public static Vector3 Value( Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, float t ){
            float oneMinusT = 1.0f - t;
            float omtSquard = oneMinusT * oneMinusT;
            float omtCubed  = omtSquard * oneMinusT;
            float tSquared  = t * t;
            float tCubed    = tSquared * t;
            return omtCubed*P0 + 3.0f*omtSquard*t*P1 + 3*oneMinusT*tSquared*P2 + tCubed*P3;
        }

        Vector3 p0 = P0;
        Vector3 p1 = P1;
        Vector3 p2 = P2; 
        Vector3 p3 = P3;


        public override Vector3 Val( float t ){  return Value( p0, p1, p2, p3, t );  }


    }

}



}
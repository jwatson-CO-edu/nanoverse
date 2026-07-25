using OpenTK.Mathematics;

namespace spline {

/// <summary>
/// Abstract base class for all Parametric curves 
/// </summary>
public abstract class Parametric {
    public abstract Vector3 Val( float t ); // Value
    public abstract Vector3 Tan( float t ); // Tangent
    public abstract Vector3 Crv( float t ); // Curvature
}


public class DummyCurve : Parametric {
    public override Vector3 Val( float t ){  return new Vector3();  }
    public override Vector3 Tan( float t ){  return new Vector3();  }
    public override Vector3 Crv( float t ){  return new Vector3();  }
}


/// <summary>
/// Bezier Curves (Component function of a B-Spline)
/// </summary>
public class Bezier {


    /// <summary>
    /// Quadratic Bezier Curve
    /// </summary>
    public class Quad ( Vector3 P0, Vector3 P1, Vector3 P2 ) : Parametric {

        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public static Vector3 Value( Vector3 P0, Vector3 P1, Vector3 P2, float t ){
            float oneMinusT = 1.0f - t;
            float omtSquard = oneMinusT * oneMinusT;
            float tSquared  = t * t;
            return omtSquard*P0 + 2*oneMinusT*t*P1 + tSquared*P2;
        }

        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public static Vector3 Tangent( Vector3 P0, Vector3 P1, Vector3 P2, float t ){
            float oneMinusT = 1.0f - t;
            return 2*oneMinusT*(P1 - P0) + 2*t*(P2 - P1);
        }


        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public static Vector3 Curvature( Vector3 P0, Vector3 P1, Vector3 P2 ){
            return 2*(P2 - 2*P1 + P0);
        }

        public Vector3 p0 = P0;
        public Vector3 p1 = P1;
        public Vector3 p2 = P2; 

        
        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public override Vector3 Val( float t ){  return Value( p0, p1, p2, t );  }
        
        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public override Vector3 Tan( float t ){  return Tangent( p0, p1, p2, t );  }
        
        
        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public override Vector3 Crv( float t ){  return Curvature( p0, p1, p2 );   }
    }


    /// <summary>
    /// Cubic Bezier Curve
    /// </summary>
    public class Cubic ( Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3 ) : Parametric {

        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public static Vector3 Value( Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, float t ){
            float oneMinusT = 1.0f - t;
            float omtSquard = oneMinusT * oneMinusT;
            float omtCubed  = omtSquard * oneMinusT;
            float tSquared  = t * t;
            float tCubed    = tSquared * t;
            return omtCubed*P0 + 3.0f*omtSquard*t*P1 + 3*oneMinusT*tSquared*P2 + tCubed*P3;
        }

        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public static Vector3 Tangent( Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, float t ){
            float oneMinusT = 1.0f - t;
            float omtSquard = oneMinusT * oneMinusT;
            float tSquared  = t * t;
            return 3*omtSquard*(P1 - P0) + 6*oneMinusT*t*(P2 - P1) + 3*tSquared*(P3 - P2);
        }

        
        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public static Vector3 Curvature( Vector3 P0, Vector3 P1, Vector3 P2, Vector3 P3, float t ){
            float oneMinusT = 1.0f - t;
            return 6*oneMinusT*(P2 - 2*P1 + P0) + 6*t*(P3 - 2*P2 + P1);
        }

        Vector3 p0 = P0;
        Vector3 p1 = P1;
        Vector3 p2 = P2; 
        Vector3 p3 = P3;


        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public override Vector3 Val( float t ){  return Value( p0, p1, p2, p3, t );  }
        
        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public override Vector3 Tan( float t ){  return Tangent( p0, p1, p2, p3, t );  }
        
        
        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public override Vector3 Crv( float t ){  return Curvature( p0, p1, p2, p3, t );  }
    }
}



}
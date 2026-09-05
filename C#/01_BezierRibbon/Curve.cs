using OpenTK.Mathematics;
using geo3d;
using pose3d;

namespace parametric {

/// <summary>
/// Abstract base class for all Parametric curves 
/// </summary>
public abstract class Parametric {
    public abstract Vector3 Val( float t ); // Value
    public abstract Vector3 Tan( float t ); // Tangent
    public abstract Vector3 Crv( float t ); // Curvature

    private static int /*------*/ nextID = 0;
    public int /*--------------*/ id     = NextID();
    public LinkedList<Parametric> edges  = [];


    /// <summary>
    /// Get the ID of the next stroke
    /// </summary>
    public static int NextID(){
        int rtn = nextID;
        nextID++;
        return rtn;
    }


    // <summary>
    /// Create a bi-directional edge between `this` and `other`
    /// </summary>
    public void ConnectBidir( Parametric other ){
        edges.AddLast( other );
        other.edges.AddLast( this );
    }


    /// <summary>
    /// Is `this` connected to `other`?
    /// </summary>
    public bool HasNeighbor( Parametric other ){
        foreach( Parametric neighbor in edges ){  if( neighbor.id == other.id ){  return true;  }  }
        return false;
    }
}



/// <summary>
/// Empty "Curve"
/// </summary>
public class DummyCurve : Parametric {
    public override Vector3 Val( float t ){  return new Vector3();  }
    public override Vector3 Tan( float t ){  return new Vector3();  }
    public override Vector3 Crv( float t ){  return new Vector3();  }
}



/// <summary>
/// Straight Curves 
/// </summary>
public class Line {

    /// <summary>
    /// Line Segment
    /// </summary>
    public class Segment ( Vector3 p0, Vector3 p1 ) : Parametric {

        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public static Vector3 Value( Vector3 P0, Vector3 P1, float t ){  return P0*(1 - t) + P1*t;  }


        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public static Vector3 Tangent( Vector3 P0, Vector3 P1 ){  return P1 - P0;  }


        /// <summary>
        /// Curvature is Zero, Return perpendicular direction
        /// </summary>
        public static Vector3 Curvature( Vector3 P0, Vector3 P1 ){
            // norm = norm.Normalized();
            Vector3 perp = new(0,0,1);
            if( MathVec3.Equal( Tangent( P0, P1 ).Normalized(), perp ) ){  perp = new(0,1,0);  }
            return Vector3.Cross( perp, Tangent( P0, P1 ) ).Normalized();
        }


        public Vector3 p0 = p0;
        public Vector3 p1 = p1;


        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public override Vector3 Val( float t ){  return Value( p0, p1, t );  }
        
        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public override Vector3 Tan( float t ){  return Tangent( p0, p1 );  }
        
        
        /// <summary>
        /// Curvature is Zero, Return perpendicular direction
        /// </summary>
        public override Vector3 Crv( float t ){  return Curvature( p0, p1 ); }
    } 
}



/// <summary>
/// Elliptical Curves 
/// </summary>
public class Ellipse {

    /// <summary>
    /// Perfect Circle 
    /// </summary>
    public class Circle ( Vector3 cntr, Vector3 norm, float radius ) : Parametric {

        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public static Vector3 Value( Vector3 cntr, Vector3 norm, float radius, float t ){
            // norm = norm.Normalized();
            Vector3 perp = new(1,0,0);
            if( MathVec3.Equal( norm, perp ) ){  perp = new(0,1,0);  }
            Vector3    begin = Vector3.Cross( norm, perp ).Normalized() * radius;
            float /**/ theta = t * 2.0f * (float) Math.PI;
            Quaternion rottn = MathVec3.AxisAngleQuat( norm, theta );
            return cntr + rottn * begin;
        }

        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public static Vector3 Tangent( Vector3 norm, float radius, float t ){
            // norm = norm.Normalized();
            Vector3 perp = new(1,0,0);
            if( MathVec3.Equal( norm, perp ) ){  perp = new(0,1,0);  }
            Vector3    begin = Vector3.Cross( norm, perp );
            float /**/ theta = t * 2.0f * (float) Math.PI;
            Quaternion rottn = MathVec3.AxisAngleQuat( norm, theta );
            return Vector3.Cross( norm, rottn * begin ).Normalized() * radius;
        }


        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public static Vector3 Curvature( Vector3 norm, float radius, float t ){
            // norm = norm.Normalized();
            return Vector3.Cross( norm, Tangent( norm, radius, t ) ).Normalized() * (1.0f / radius);
        }


        public Vector3 cntr   = cntr;
        public Vector3 norm   = norm;
        public float   radius = radius;


        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public override Vector3 Val( float t ){  return Value( cntr, norm, radius, t );  }
        
        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public override Vector3 Tan( float t ){  return Tangent( norm, radius, t );      }
        
        
        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public override Vector3 Crv( float t ){  return Curvature( norm, radius, t );    }
    }


    /// <summary>
    /// Perfect Circle 
    /// </summary>
    public class Oval ( Vector3 cntr, Vector3 norm, Vector3 begin, float aRad, float bRad ) : Parametric {

        public Vector3 cntr  = cntr;
        public Vector3 norm  = norm;
        public Vector3 begin = begin;
        public float   aRad  = aRad;
        public float   bRad  = bRad;

        /// <summary>
        /// Radial equation for an ellipse
        /// </summary>
        public static float OvalRad( float aRad, float bRad, float theta ){
            return aRad * bRad / MathF.Sqrt( MathF.Pow( aRad*MathF.Cos( theta ), 2f ) + MathF.Pow( bRad*MathF.Sin( theta ), 2f ) );
        }

        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public static Vector3 Value( Vector3 cntr, Vector3 norm, Vector3 begin, float aRad, float bRad, float t ){
            Matrix4 basis = MathMatx4.HomogFromXZBases( begin, norm, cntr );
            Vector3 xDir  = MathMatx4.GetXBasis( basis );
            float   theta = t*2f*MathF.PI;
            float   tRad  = OvalRad( aRad, bRad, theta );
            return cntr + MathVec3.AxisAngleQuat( norm, theta ) * xDir * tRad;
        }


        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public static Vector3 Tangent( Vector3 cntr, Vector3 norm, Vector3 begin, float aRad, float bRad, float t ){
            Vector3 pnt = Value( cntr, norm, begin, aRad, bRad, t );
            Vector3 ray = pnt - cntr;
            return  Vector3.Cross( ray, norm ).Normalized();
        }


        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public static Vector3 Curvature( Vector3 cntr, Vector3 norm, Vector3 begin, float aRad, float bRad, float t ){
            Vector3 pnt = Value( cntr, norm, begin, aRad, bRad, t );
            return (cntr - pnt).Normalized(); 
        }

        
        /// <summary>
        /// Point on the curve for parameter `t`
        /// </summary>
        public override Vector3 Val( float t ){
            return Value( cntr, norm, begin, aRad, bRad, t );
        }

        
        /// <summary>
        /// Tangent on the curve for parameter `t`
        /// </summary>
        public override Vector3 Tan( float t ){
            return Tangent( cntr, norm, begin, aRad, bRad, t );
        }


        /// <summary>
        /// Curvature at parameter `t`
        /// </summary>
        public override Vector3 Crv( float t ){
            return Curvature( cntr, norm, begin, aRad, bRad, t );
        }
    }

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

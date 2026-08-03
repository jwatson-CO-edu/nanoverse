using OpenTK.Mathematics;


/// <summary>
/// Helper functions for this project
/// </summary>
namespace helpers {


/// <summary>
/// Working with `OpenTK.Mathematics.Vector3`
/// </summary>
public class MathVec3 {

    const float _EPSILON = 0.00001f;

    
    /// <summary>
    /// Are the vectors equal?
    /// </summary>
    public static bool Equal( Vector3 vec1 , Vector3 vec2 ){  return (vec1 - vec2).Length < _EPSILON;  }


    /// <summary>
    /// Get the angle between two R3 vectors , radians
    /// </summary>
    public static float AngleBetween( Vector3 vec1 , Vector3 vec2 ){
        float angle = (float) Math.Acos( Vector3.Dot( vec1.Normalized(), vec2.Normalized() ) ); // for now assume that there are no special cases
        if( float.IsNaN( angle ) ){
            if( Equal( vec1.Normalized(), vec2.Normalized() ) ){ return 0.0f; }
            else { return (float) Math.PI; }
        } else { return angle; }
    }


    /// <summary>
    /// `Quaternion.FromAxisAngle`, enforce normalized axis
    /// </summary>
    public static Quaternion AxisAngleQuat( Vector3 axis, float theta ){
        return Quaternion.FromAxisAngle( axis.Normalized(), theta );
    }


    /// <summary>
    /// Generate a 3D vector with random {X,Y,} components of a specified `scale`
    /// </summary>
    public static Vector3 NoiseXY( Random rand, float scale = 1.0f ){
        return new Vector3( rand.NextSingle(), rand.NextSingle(), 0.0f ).Normalized() * scale;
    }


}





}
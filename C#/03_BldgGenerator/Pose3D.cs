using geo3d;
using OpenTK.Mathematics;


/// <summary>
/// Helper functions for 3D Geometry
/// </summary>
namespace pose3d {


/// <summary>
/// Working with `OpenTK.Mathematics.Matrix4`,
/// OpenTK's Matrix4 is row-major, and it uses the row-vector convention (v' = v * M, not M * v)
/// </summary>
public static class MathMatx4 {

    /* https://learn.microsoft.com/en-us/previous-versions/windows/xna/bb197911(v=xnagamestudio.42)
    
       When you later compose transforms, remember OpenTK's multiplication order reads left-to-right 
       in application order — child * parent, not parent * child as in GLM/HLSL math — and when you 
       finally ship this to a GLSL uniform via GL.UniformMatrix4, you'll need transpose: true unless 
       you're using OpenTK's own matrix math consistently on both ends. 
       
       Matrices use a row vector layout in the XNA Framework. Matrices can be either row vector or 
       column vector. Row vector matrices view vectors as a row from left to right, while column vector 
       matrices view vectors as a column from top to bottom. For example, the x, y, and z of a matrix's 
       translation vector in the XNA Framework would correspond to the fields M41, M42, M43. */


    /// <summary>
    /// Return the Homogeneous Coordinates from the basis vectors and position, ROW MAJOR
    /// </summary>
    public static Matrix4 HomogFromBases( Vector3 xDir, Vector3 yDir, Vector3 zDir, Vector3 posn ){
        Matrix4 homog = new(
            xDir.X, xDir.Y, xDir.Z, 0f, // Row0: local X axis, in world space
            yDir.X, yDir.Y, yDir.Z, 0f, // Row1: local Y axis
            zDir.X, zDir.Y, zDir.Z, 0f, // Row2: local Z axis
            posn.X, posn.Y, posn.Z, 1f  // Row3: translation 
        );
        return homog;
    }


    /// <summary>
    /// Return the Homogeneous Coordinates from the X and Z basis vectors and position, Z has primacy
    /// </summary>
    public static Matrix4 HomogFromXZBases( Vector3 xDir, Vector3 zDir, Vector3 posn ){
        Vector3 yDir;
        yDir = Vector3.Cross( zDir, xDir ).Normalized(); // Find the Y direction
        xDir = Vector3.Cross( yDir, zDir ).Normalized(); // Enforce orthonormal X direction
        zDir = zDir.Normalized(); // ---------------------- Enforce unit Z direction
        return HomogFromBases( xDir, yDir, zDir, posn );
    }


    /// <summary>
    /// Return the local X-direction for the Homogeneous Coordinates 
    /// </summary>
    public static Vector3 GetXBasis( Matrix4 homog ){  return new Vector3( homog.M11, homog.M12, homog.M13 );  }
    

    /// <summary>
    /// Return the local Y-direction for the Homogeneous Coordinates 
    /// </summary>
    public static Vector3 GetYBasis( Matrix4 homog ){  return new Vector3( homog.M21, homog.M22, homog.M23 );  }
    

    /// <summary>
    /// Return the local Z-direction for the Homogeneous Coordinates 
    /// </summary>
    public static Vector3 GetZBasis( Matrix4 homog ){  return new Vector3( homog.M31, homog.M32, homog.M33 );  }
    

    /// <summary>
    /// Return the position for the Homogeneous Coordinates 
    /// </summary>
    public static Vector3 GetPosition( Matrix4 homog ){  return new Vector3( homog.M41, homog.M42, homog.M43 );  }


    /// <summary>
    /// Return a transformed copy of the mesh, ASSUMPTION: AFFINE TRANSFORMATION 
    /// </summary>
    public static List<Tri> TransformMesh( List<Tri> mesh, Matrix4 xform ){
        List<Tri> movMsh = Tri.CopyMesh( mesh );
        foreach( Tri tri in movMsh ){
            tri[0] = new Vector3( new Vector4( tri[0], 1f ) * xform );
            tri[1] = new Vector3( new Vector4( tri[1], 1f ) * xform );
            tri[2] = new Vector3( new Vector4( tri[2], 1f ) * xform );
        }
        return movMsh;
    }
}


}
using OpenTK.Mathematics;

using geo3d;


namespace ribbon;


/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////
[ ] Parametric Arc
[ ] Ribbon Arc


*/


public class LatinStroke : IMeshHaver {
    
    /// Creation ///
    public RNode /*-------*/ bgn /**/ = new();
    public RNode /*-------*/ end /**/ = new();
    public RNode /*-------*/ center   = new();
    public Vector3 /*-----*/ mainAxis = Vector3.UnitY;
    public List<LatinStroke> edges    = [];
    public Ribbon /*------*/ stroke   = new();

    /// Connection ///
    public void AddEdge( LatinStroke neighbor ){  edges.Add( neighbor );  }

    /// Mesh ///
    public List<Tri> GetTotalMesh(){  return stroke.GetTotalMesh();  }

}


/// <summary>
/// Generate a glyph with Latin components and patterns
/// </summary>
public class LatinGlyph {



}
using OpenTK.Mathematics;

using geo3d;


namespace ribbon;


/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////
* Parent Class
    [ ] "Parent" Parameter `Val`, Meta-`t`
    [ ] Meta-`Tan`
[ ] Parametric Arc
[ ] Ribbon Arc


*/


/// <summary>
/// Component of a Latin-style glyph
/// </summary>
public class LatinStroke : IMeshHaver {
    
    ///// Members /////
    public RNode /*-------*/ bgn /**/ = new();
    public RNode /*-------*/ end /**/ = new();
    public RNode /*-------*/ center   = new();
    public Vector3 /*-----*/ mainAxis = Vector3.Zero;
    public List<LatinStroke> edges    = [];
    public List<Ribbon> /**/ strokes  = [];


    ///// Methods /////
    
    /// Connection Methods ///
    public bool HasAxis(){  return mainAxis.Length > MathVec3._EPSILON;  }
    public void AddEdge( LatinStroke neighbor ){  edges.Add( neighbor );  }
    public virtual int Occupancy(){  return edges.Count;  }


    /// Mesh Methods ///
    public List<Tri> GetTotalMesh(){  
        List<Tri> rtnMsh = [];
        foreach( Ribbon stroke in strokes ){  rtnMsh.AddRange( stroke.GetTotalMesh() );  }
        return rtnMsh;  
    }

}



// QWERTYUIOPASDFGHJKLZXCVBNM
// qwertyuiopasdfghjklzxcvbnm


/// <summary>
/// Container for Latin glyph fragments
/// </summary>
public static class Latin {

    public const float aFctr = 1f;
    public const float bFctr = 1.3f;

    /// <summary>
    /// Any closed ellipse in a glyph
    /// </summary>
    public class Ellipse ( Vector3 center, Vector3 norm, Vector3 begin, float scale ) : LatinStroke {

    }


    /// <summary>
    /// Any elliptical bend in a glyph
    /// </summary>
    public class ArcEllp : LatinStroke {

    }


    /// <summary>
    /// Any straight structure in a glyph
    /// </summary>
    public class Spar : LatinStroke {

    }


    /// <summary>
    /// Quadratic Bezier
    /// </summary>
    public class QBez : LatinStroke {

    }


    /// <summary>
    /// Cubic Bezier
    /// </summary>
    public class CBez : LatinStroke {

    }


    /// <summary>
    /// Fancy terminator that prevents further connection
    /// </summary>
    public class Serif : LatinStroke {
        public override int Occupancy(){  return int.MaxValue;  }
    }


    /// <summary>
    /// Floating fragments that should not be connected to
    /// </summary>
    public class Kipple : LatinStroke {
        public override int Occupancy(){  return int.MaxValue;  }
    }


}


/// <summary>
/// Generate a glyph with Latin components and patterns, Build rules here
/// </summary>
public class LatinGlyph { // : IMeshHaver 
    // TODO: DON'T FORGET TO ADD OVERLAPS
    // NOTE: Italics are NOT modeled!


}
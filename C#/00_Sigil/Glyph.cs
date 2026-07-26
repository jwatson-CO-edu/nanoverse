using OpenTK.Mathematics;
using curve;

namespace sigil {


/// <summary>
/// Triangle
/// </summary>
public readonly struct Tri{
    private readonly Vector3[] verts = new Vector3[3]; // Array ref cannot change, values can 

    
    /// <summary>
    /// Three points as a Triangle
    /// </summary>
    public Tri( Vector3 a, Vector3 b, Vector3 c ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
    }


    /// <summary>
    /// Indexer: {get, set,}
    /// </summary>
    public readonly Vector3 this[int index]{
        get => verts[ index ];
        set => verts[ index ] = value;
    }


    /// <summary>
    /// First point (CCW)
    /// </summary>
    public readonly Vector3 V0() => verts[0];
    

    /// <summary>
    /// Second point (CCW)
    /// </summary>
    public readonly Vector3 V1() => verts[1];
    
    
    /// <summary>
    /// Third point (CCW)
    /// </summary>
    public readonly Vector3 V2() => verts[2];
}



/// <summary>
/// A curve with thickness in 3D space
/// </summary>
public class Stroke {
    public const int  _DEFAULT_DIV = 32;
    public float /**/ thick;
    public Parametric curve;
    public int /*--*/ div;
    public List<Tri>  geo;

    
    /// <summary>
    /// Default constructor
    /// </summary>
    public Stroke(){
        thick = 0.0f;
        curve = new DummyCurve();
        div   = _DEFAULT_DIV;
        geo   = [];
    }

    
    /// <summary>
    /// Set curve and thickness
    /// </summary>
    public Stroke( Parametric param, float thickness, int div_ = _DEFAULT_DIV ){
        thick = thickness;
        curve = param;
        div   = div_;
        geo   = [];
        geo.Capacity = 2*div;
    }


    /// <summary>
    /// Create mesh for drawing
    /// </summary>
    public void BuildGeo(){

    }
}





}
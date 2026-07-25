using OpenTK.Mathematics;
using spline;

namespace sigil {


public readonly struct Tri{
    private readonly Vector3[] verts = new Vector3[3]; // Array ref cannot change, values can 

    public Tri( Vector3 a, Vector3 b, Vector3 c ){
        verts[0] = a;
        verts[1] = b;
        verts[2] = c;
    }

    public readonly Vector3 this[int index]{
        get => verts[ index ];
        set => verts[ index ] = value;
    }

    public readonly Vector3 V0() => verts[0];
    public readonly Vector3 V1() => verts[1];
    public readonly Vector3 V2() => verts[2];
}



public class Stroke {
    public const int  _DEFAULT_DIV = 32;
    public float /**/ thick;
    public Parametric curve;
    public int /*--*/ div;
    public List<Tri>  geo;

    public Stroke(){
        thick = 0.0f;
        curve = new DummyCurve();
        div   = _DEFAULT_DIV;
        geo   = [];
    }

    public Stroke( Parametric param, float thickness, int div_ = _DEFAULT_DIV ){
        thick = thickness;
        curve = param;
        div   = div_;
        geo   = [];
        geo.Capacity = 2*div;
    }
}





}
using geo3d;
using OpenTK.Mathematics;

namespace ideogram {

public class MeshGen {

    public Random rand = new();

    public List<Tri> Cylinder( Vector3 center, Vector3 axis, float radius = 1f, float height = 1f, int arcDiv = 16, int hgtDiv = 2 ){
        List<Tri> rtnShp = [];
        axis.Normalize();
        Vector3 top  = center + axis * (height * 0.5f);
        Vector3 btm  = center - axis * (height * 0.5f);
        Vector3 zSpr = Vector3.Cross( axis, MathVec3.NoiseXYZ( rand ) ).Normalized() * radius;
        Vector3 spar;
        Vector3 lSpr = zSpr;

        // FIXME: START HERE - GENERATE A CYLINDER WITH THE GIVEN PARAMS
        // FIXME: GENERATE TOP
        // FIXME: GENERATE BOTTOM
        // FIXME: GENERATE BANDS

        return rtnShp;
    }

}

public class IdeoComponent {
    public List<Tri>     mesh  = [];
    public List<Matrix4> ports = [];
}

}
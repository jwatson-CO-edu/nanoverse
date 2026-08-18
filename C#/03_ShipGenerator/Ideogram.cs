using geo3d;
using OpenTK.Mathematics;

namespace ideogram {

public class MeshGen {

    public Random rand = new();

    public List<Tri> Cylinder( Vector3 center, Vector3 axis, float radius = 1f, float height = 1f, int arcDiv = 16, int hgtDiv = 2 ){
        List<Tri>     rtnShp = [];
        List<Vector3> segCtr = [];
        rtnShp.Capacity = 2 * arcDiv + 2 * arcDiv * hgtDiv; // 2x Endcaps + `arcDiv` x `hgtDiv` Quads
        segCtr.Capacity = hgtDiv + 1;
        axis.Normalize();
        Vector3 top = center + axis * (height * 0.5f);
        Vector3 btm = center - axis * (height * 0.5f);
        segCtr.Add( btm );
        Vector3 zSpr = Vector3.Cross( axis, MathVec3.NoiseXYZ( rand ) ).Normalized() * radius;
        Vector3 spar, p0, p1, p2, p3;
        Vector3 lSpr = zSpr;
        float   dTheta = 2f * MathF.PI / arcDiv;
        float   dHgt   = height / hgtDiv;

        for( int j = 1; j < hgtDiv; ++j ){  segCtr.Add( btm + axis * j *  dHgt );  }

        for( int i = 1; i <= arcDiv; ++i ){
            // Rotate radial spar
            spar = MathVec3.AxisAngleQuat( axis, dTheta ) * lSpr;
            // Generate Top `Tri`
            rtnShp.Add( new Tri( top, top + lSpr, top + spar ) );
            // Generate Bottom `Tri`
            rtnShp.Add( new Tri( btm, btm + spar, btm + lSpr ) );
            // Generate longitudinal quad strip
            for( int j = 1; j <= hgtDiv; ++j ){
                p0 = segCtr[j-1] + lSpr;
                p1 = segCtr[j-1] + spar;
                p2 = segCtr[j  ] + lSpr;
                p3 = segCtr[j  ] + spar;
                rtnShp.Add( new Tri( p0, p1, p2 ) );
                rtnShp.Add( new Tri( p2, p3, p0 ) );
            }
            lSpr = spar;
        }
        

        return rtnShp;
    }

}

public class IdeoComponent {
    public List<Tri>     mesh  = [];
    public List<Matrix4> ports = [];
}

}
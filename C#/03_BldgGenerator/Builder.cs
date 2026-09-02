using OpenTK.Mathematics;
using geo3d;


namespace ideogram {

/// <summary>
/// Builder pattern for building meshes
/// </summary>
public static class Builder {

    public static List<Tri> StackedApartment( Vector3 centerBase, Vector3 color, int Nunit, 
                                              float wideWidth, float wideHeight, float narrowWidth, float narrowHeight ){
        List<Tri> totMsh = [];
        Vector3   base_i = centerBase;
        List<Tri> mesh_i;

        for( int i = 0; i < Nunit; ++i ){

            base_i[3] += narrowHeight/2;
            mesh_i = MeshGen.Cuboid( base_i, narrowWidth, narrowWidth, narrowHeight );
            totMsh.AddRange( mesh_i );

            base_i[3] += narrowHeight/2 + wideHeight/2;
            mesh_i = MeshGen.Cuboid( base_i, wideWidth, wideWidth, wideHeight );
            totMsh.AddRange( mesh_i );

            base_i[3] += wideHeight/2;
            
        }

        return Tri.ColorMesh( totMsh, new Vector4( color, 1f ) ) ;
    }

}

}
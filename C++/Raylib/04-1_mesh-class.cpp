#include <vector>
using std::vector;

#include <raylib.h>
#include <rlgl.h>

typedef unsigned short ushort;
typedef unsigned int   uint;

// static Mesh gen_mesh( int n );

// static Mesh gen_mesh( Mesh& mesh, int n ){
void gen_mesh( Mesh& mesh, int n ){
    // Mesh mesh = { 0 };
    float triangle[9] = {
        0.f, 0.f, 0.0f, 
        1.f, 0.f, 0.0f, 
        0.f, 1.f, 0.0f
    };
    // Mesh mesh{};
    mesh.triangleCount = n;
    mesh.vertexCount   = n * 3;
    // mesh.vertices /**/ = new float[mesh.vertexCount * 3];
    // mesh.indices /*-*/ = new unsigned short[mesh.vertexCount];
    mesh.vertices /**/ = (float *)MemAlloc(mesh.vertexCount*3*sizeof(float)); // 3 vertices, 3 coordinates each (x, y, z)
    mesh.indices /*-*/ = (ushort *)MemAlloc(mesh.vertexCount*sizeof(ushort));
    
    for( uint j = 0; j < 9; ++j ) /*----*/ mesh.vertices[j] = triangle[j];
    for( ushort ix = 0; ix < n * 3; ++ix)  mesh.indices[ix] = ix;

    // UploadMesh(&mesh, true);
    // return mesh;
}

struct TriStore{
    // Obviously meshes aren't meant to be kept around ??????
    Model model;
    int   n;

    TriStore( int n_ = 1 ){
        n     = n_;
        // model = LoadModelFromMesh( gen_mesh(n) );
    }
};

int main(){
    // TriStore tri{};
    // tri.model = LoadModelFromMesh( gen_mesh(1) );
    Model model;
    Mesh  mesh{};

    InitWindow(800, 600, "Dynamic Mesh test");

    gen_mesh(mesh,1);
    UploadMesh(&mesh, true); // THIS MUST HAPPEN AFTER WINDOW INIT!
    model = LoadModelFromMesh( mesh );
    return 0;
}


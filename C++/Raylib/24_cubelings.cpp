// g++ 24_cubelings.cpp -std=c++17 -lraylib -O3
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

class Cube : public DynaMesh{ public:
    // Simple Cube

    /// Constructors ///

    Cube( float sideLen, Color color ) : DynaMesh( 12 ){
        // Build and save geo
        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm;
        vvec3   V;
        float   halfLen = sideLen/2.0;

        // 1. Establish vertices
        V.push_back( Vector3{ -halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen,  halfLen } );

        // 2. Build tris
        push_triangle_w_norms( { V[0], V[3], V[2] } );
        push_triangle_w_norms( { V[0], V[1], V[3] } );
        push_triangle_w_norms( { V[6], V[4], V[0] } );
        push_triangle_w_norms( { V[6], V[0], V[2] } );
        push_triangle_w_norms( { V[0], V[4], V[5] } );
        push_triangle_w_norms( { V[0], V[5], V[1] } );
        push_triangle_w_norms( { V[7], V[6], V[2] } );
        push_triangle_w_norms( { V[7], V[2], V[3] } );
        push_triangle_w_norms( { V[4], V[6], V[7] } );
        push_triangle_w_norms( { V[4], V[7], V[5] } ); 
        push_triangle_w_norms( { V[1], V[5], V[7] } );
        push_triangle_w_norms( { V[1], V[7], V[3] } );

        // 3. Set color
        set_uniform_color( color );
    }
};

class Cylinder : public DynaMesh{ public:
    // Simple Cylinder
    // WARNING: Instantiation REQUIRES a graphics context?

    /// Constructors ///

    Cylinder( float radius, float length, Color color ){
        // Load mesh data from default Raylib cylinder
        mesh = GenMeshCylinder( radius, length, 8 ); // Created cylinder with 144 triangles and 432 vertices!
        cout << "Generated cylinder!" << endl;
        Ntri = mesh.triangleCount;
        Nvtx = mesh.vertexCount;
        cout << "Created cylinder with " << Ntri << " triangles and " << Nvtx << " vertices!" << endl;
    }
};

// FIXME, START HERE: CREATE A `Cubeling` CLASS WITH OSCILLATING PEGS ON EACH FACE

// FIXME: CREATE A POPULATION OF WANDERING CUBELINGS WITH SIMPLE BEHAVIOR AND DISPLAY 



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "L-System Test" );
    SetTargetFPS( 60 );

    ///// Create Objects //////////////////////////////////////////////////

    Cube cube( 1.0, DARKBLUE );
    Cylinder cyl{ 1.0, 1.0, DARKBLUE };

    return 0;
}

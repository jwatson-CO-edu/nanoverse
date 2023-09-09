// g++ 21_L-Blocks.cpp -std=c++17 -lraylib -O3
// Building as an L-System


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp"



////////// DRAWABLE ////////////////////////////////////////////////////////////////////////////////

class Cuboid : public DynaMesh{
    // Axis-aligned cuboid with centroid at <0,0,0>

    vvec3 vrts;
    Color colr;

    Cuboid( float xLen, float yLen, float zLen, Color color ) : DynaMesh( 12 ){
        // Generate and store geometry with a uniform color

        float xHalf = xLen/2.0;
        float yHalf = yLen/2.0;
        float zHalf = zLen/2.0;

        // 1. Establish vertices
        vrts.push_back( Vector3{ -xHalf, -yHalf, -zHalf } );
        vrts.push_back( Vector3{ -xHalf, -yHalf,  zHalf } );
        vrts.push_back( Vector3{ -xHalf,  yHalf, -zHalf } );
        vrts.push_back( Vector3{ -xHalf,  yHalf,  zHalf } );
        vrts.push_back( Vector3{  xHalf, -yHalf, -zHalf } );
        vrts.push_back( Vector3{  xHalf, -yHalf,  zHalf } );
        vrts.push_back( Vector3{  xHalf,  yHalf, -zHalf } );
        vrts.push_back( Vector3{  xHalf,  yHalf,  zHalf } );

        // 2. Build tris
        push_triangle_w_norms( { vrts[0], vrts[3], vrts[2] } );
        push_triangle_w_norms( { vrts[0], vrts[1], vrts[3] } );
        push_triangle_w_norms( { vrts[6], vrts[4], vrts[0] } );
        push_triangle_w_norms( { vrts[6], vrts[0], vrts[2] } );
        push_triangle_w_norms( { vrts[0], vrts[4], vrts[5] } );
        push_triangle_w_norms( { vrts[0], vrts[5], vrts[1] } );
        push_triangle_w_norms( { vrts[7], vrts[6], vrts[2] } );
        push_triangle_w_norms( { vrts[7], vrts[2], vrts[3] } );
        push_triangle_w_norms( { vrts[4], vrts[6], vrts[7] } );
        push_triangle_w_norms( { vrts[4], vrts[7], vrts[5] } ); 
        push_triangle_w_norms( { vrts[1], vrts[5], vrts[7] } );
        push_triangle_w_norms( { vrts[1], vrts[7], vrts[3] } );

        // 3. Set color
        set_uniform_color( color );

        // 4. Load buffers
        load_mesh_buffers( true, true );
    }
};

class Wedge : public DynaMesh{
    // Right-Triangular prism with axis-aligned rectangular faces pointing in +Y and -Z

    Wedge( float xLen, float yLen, float zLen, Color color ) : DynaMesh( 8 ){
        // Generate and store geometry with a uniform color
        
        float xHalf = xLen/2.0;
        float yHalf = yLen/2.0;
        float zHalf = zLen/2.0;

        // 1. Establish vertices
        vrts.push_back( Vector3{ -xHalf, -yHalf, -zHalf } );
        vrts.push_back( Vector3{ -xHalf,  yHalf, -zHalf } );
        vrts.push_back( Vector3{ -xHalf,  yHalf,  zHalf } );
        vrts.push_back( Vector3{  xHalf, -yHalf, -zHalf } );        
        vrts.push_back( Vector3{  xHalf,  yHalf, -zHalf } );
        vrts.push_back( Vector3{  xHalf,  yHalf,  zHalf } );

        // 2. Build tris
        push_triangle_w_norms( { vrts[0], vrts[2], vrts[1] } );
        push_triangle_w_norms( { vrts[4], vrts[3], vrts[0] } );
        push_triangle_w_norms( { vrts[4], vrts[0], vrts[1] } );
        push_triangle_w_norms( { vrts[5], vrts[4], vrts[1] } );
        push_triangle_w_norms( { vrts[5], vrts[1], vrts[2] } );
        push_triangle_w_norms( { vrts[3], vrts[4], vrts[5] } );
        push_triangle_w_norms( { vrts[2], vrts[0], vrts[3] } );
        push_triangle_w_norms( { vrts[3], vrts[5], vrts[2] } );

        // 3. Set color
        set_uniform_color( color );

        // 4. Load buffers
        load_mesh_buffers( true, true );
    }
};



////////// LINDENMAYER SYSTEM //////////////////////////////////////////////////////////////////////

///// Forward Declarations /////
class L_Node;
typedef shared_ptr<L_Node> nodePtr;


///// Node ///////////////////////////////////////

class L_Rule{ public:
    // Base class for L-System transformation rules
    nodePtr node; // ------------- Node to be transformed
    virtual void transform(){}; // Transformation strategy
};
typedef shared_ptr<L_Rule> rulePtr;

class L_Node{ public:
    // Basis for a graphical Lindenmayer System
    nodePtr /*---*/ parent; // - Edge from parent
    vector<nodePtr> children; // Edges to children
    rulePtr /*---*/ rule; // --- Production rule for transforming the graph, FIXME: MANY INSTANCES OR ONE?
    Matrix /*----*/ Trel; // --- Relative transform from the parent frame
    Matrix /*----*/ Tabs; // --- Absolute transform in the world frame
    dynaPtr /*---*/ drawable; // `DynaMesh` that renders this node
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "L-System Test" );
    SetTargetFPS( 60 );

    ///// Create Objects //////////////////////////////////////////////////

    

    return 0;
}
// g++ 21_L-Blocks.cpp -std=c++17 -lraylib -O3
// Building as an L-System


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////



Matrix set_posn( const Matrix& xfrm, const Vector3& posn ){
    // Set the position components of the homogeneous coordinates
    Matrix rtnMatx{ xfrm };
    rtnMatx.m12 = posn.x;
    rtnMatx.m13 = posn.y;
    rtnMatx.m14 = posn.z;
    return rtnMatx;
}

Matrix rotate_RPY( const Matrix& xfrm, float r_, float p_, float y_ ){
    // Increment the world Roll, Pitch, Yaw of the model
    // NOTE: This is for airplanes that move forward in their own Z and have a wingspan across X
    return MatrixMultiply( 
        MatrixMultiply( MatrixMultiply( MatrixRotateY( y_ ), MatrixRotateX( p_ ) ), MatrixRotateZ( r_ ) ), 
        xfrm 
    );
}


////////// DRAWABLE ////////////////////////////////////////////////////////////////////////////////

class Cuboid : public DynaMesh{ public:
    // Axis-aligned cuboid with centroid at <0,0,0>

    /// Constructors ///
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

class Wedge : public DynaMesh{ public:
    // Right-Triangular prism with axis-aligned rectangular faces pointing in +Y and -Z

    /// Constructors ///
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
class /*----------------*/ L_Node;
typedef shared_ptr<L_Node> nodePtr;


///// Node ///////////////////////////////////////

enum NodeType{
    EMPTY, // Default node, not rendered
    TOWER, // Defensive structure, capped on top and bottom by turrets
    TURET, // Point defense
    PIVOT, // Rotates subtree about local X
};



class L_Node{ public:
    // Basis for a graphical Lindenmayer System

    /// Members ///
    NodeType /*--*/ type; // --- String designator governing rendering and rules
    float /*-----*/ unit; //- Unit length [m] for generating geometry
    nodePtr /*---*/ parent; // - Edge from parent
    vector<nodePtr> children; // Edges to children
    Matrix /*----*/ Trel; // --- Relative transform from the parent frame
    Matrix /*----*/ Tabs; // --- Absolute transform in the world frame
    dynaPtr /*---*/ drawable; // `DynaMesh` that renders this node

    /// Constructor ///
    L_Node(){
        // Empty node
        type     = EMPTY;
        parent   = nullptr;
        Trel     = MatrixIdentity();
        Tabs     = MatrixIdentity();
        drawable = nullptr;
    }

    /// Methods ///

    void add_child( nodePtr nuChild ){
        // Create a connection between nodes
        nuChild->parent = nodePtr( this );
        children.push_back( nodePtr( nuChild ) );
    }

    bool p_leaf(){  return (children.size() == 0);  } // Retur true if the node has no children

    void update_poses( Matrix parentXform ){
        // Set the pose of this node and subtree of node below
        Tabs = MatrixMultiply( Trel, parentXform );
        if( !p_leaf() ){  for( nodePtr& child : children ){  child->update_poses( Tabs );  }  }
    }

    void draw(){
        // Draw this node and all nodes beyond it
        if( drawable ){  
            drawable->xfrm = Tabs;  
            drawable->draw();
        }
        for( nodePtr& child : children ){  child->draw();  }
    }
};

///// Factories //////////////////////////////////

/*
> All nodes are on the surfaces of modules
> Modules do not need their own abstraction
> A single module of the building is drawn by one node, but has many undrawn nodes as attachment points
*/

nodePtr make_tower( nodePtr parent_,  float unit, 
                    int distance, int length, int height, int depth, int yOffset ){
    /* Towers always have a square `length` x `length` vertical cross section
       Non-zero `height` and `depth` towers are capped by regular and inverted turrets, respectively */
    nodePtr rtnNode{ new L_Node{} };
    Vector3 dims = { length*unit, length*unit, (height+depth)*unit };
    Basis   sideA;
    Matrix  matxA;
    Basis   sideB;
    Matrix  matxB;

    //// Build Internal Nodes ////
    // FIXME, START HERE: GRID THE SURFACE OF THE THE TOWER WITH NODES EXCEPT FOR THIS NODE
    if( (yOffset > 0) && (yOffset < length) ){
        sideA = Basis::origin();
        sideB = Basis::from_Xb_and_Zb( Vector3{ -1.0, 0.0, 0.0 }, Vector3{ 0.0, 0.0, 1.0 } );
        for( int i = 0; i < yOffset; ++i ){
            // Build 
        }
    }

    //// Build Render Geometry ////
    // FIXME: DRAW A BOX SUCH THAT THE NODES ARE ON THE SURFACE

}

////////// LIGHTING ////////////////////////////////////////////////////////////////////////////////

struct Lighting{
    // Container struct for light(s) and associated shader

    /// Members ///
    Shader shader;
    float  ambientColor[4];
    int    ambientLoc;
    Light  light;

    /// Constructor ///
    Lighting(){
        // Load basic lighting shader
        shader = LoadShader( TextFormat("shaders/lighting.vs") ,
                             TextFormat("shaders/lighting.fs") );
        // Get some required shader locations
        shader.locs[ SHADER_LOC_VECTOR_VIEW ] = GetShaderLocation( shader, "viewPos" );
        // Ambient light level (some basic lighting)
        ambientColor[0] = 0.25f;
        ambientColor[1] = 0.25f;
        ambientColor[2] = 0.25f;
        ambientColor[3] = 0.50f;
        ambientLoc /**/ = GetShaderLocation( shader, "ambient" );
        SetShaderValue( shader, ambientLoc, ambientColor, SHADER_UNIFORM_VEC4 );
        // Using just 1 point lights
        light = CreateLight(
            LIGHT_POINT, 
            Vector3{ 20.0, 20.0, 20.0 }, 
            Vector3Zero(), 
            Color{ 255, 255, 255, 125 }, 
            shader
        );
    }

    /// Methods ///

    void update(){  UpdateLightValues( shader, light );  } // Update the light

    void set_camera_posn( Camera& cam ){
        // Tell the shader where the camera is
        SetShaderValue( shader, shader.locs[SHADER_LOC_VECTOR_VIEW], &cam.position.x, SHADER_UNIFORM_VEC3 );
    }
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

    /// Create Camera ///
    Camera camera = Camera{
        Vector3{  15.0,  15.0, -15.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    /// Lighting ///
    Lighting lightShader{};
    lightShader.set_camera_posn( camera );

    /// Drawables ///
    Cuboid cuboid{ 2.0f, 2.0f, 3.0f, BLUE  };
    Wedge  wedge{  2.0f, 2.0f, 2.0f, GREEN };
    wedge.set_posn( Vector3{ 0.0f, 0.0f, 1.25f } );
    cuboid.set_shader( lightShader.shader );
    wedge.set_shader( lightShader.shader );

    /// L-System ///
    L_Node root{};
    root.drawable = dynaPtr( new Cuboid{ 2.0f, 2.0f, 3.0f, BLUE  } );
    root.drawable->set_shader( lightShader.shader );

    nodePtr top{ new L_Node{} };
    top->drawable = dynaPtr( new Wedge{  2.0f, 2.0f, 2.0f, GREEN } );
    top->drawable->set_shader( lightShader.shader );
    top->Trel = set_posn( top->Trel, Vector3{ 0.0f, 0.0f, 1.25f } );
    root.add_child( top );

    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        lightShader.update();
        // lightShader.set_camera_posn( camera ); // Uncomment if camera moves
        // cuboid.draw();
        // wedge.rotate_RPY( M_PI/60.0f, 0.0f, 0.0f ); // RPY seems out of order?
        // wedge.draw();
        root.children[0]->Trel = rotate_RPY( root.children[0]->Trel, M_PI/60.0f, 0.0f, 0.0f );
        // root.update_poses( rotate_RPY( root.Tabs, M_PI/120.0f, M_PI/120.0f, M_PI/120.0f ) );
        root.update_poses( MatrixIdentity() );
        root.draw();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}
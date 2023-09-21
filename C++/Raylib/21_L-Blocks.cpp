// g++ 21_L-Blocks.cpp -std=c++17 -lraylib -O3
// Building as an L-System


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp"


////////// HELPER FUNCTIONS ////////////////////////////////////////////////////////////////////////

// FIXME: MOVE THESE TO THE TOYBOX




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
    // Plain Nodes //
    EMPTY, // [ ] Default node -or- surface connections, Not rendered
    TOWER, // [ ] Defensive structure, Module capped on top and bottom by pivots for turrets
    BLOCK, // [ ] Large body of classrooms and labs
    BRIDG, // [ ] Walkway between modules
    // Special Nodes / Leaves //
    WNDOW, // [ ] Window, Always a leaf
    CELLA, // [ ] Cellular antenna, Always a leaf
    RADAR, // [ ] Antenna dish, Always a leaf
    COLUM, // [ ] Support below a module (Allow space between stacked modules?)
    DECOR, // [ ] Sculptures and decorations, Can only have its own type as children
    // Active Nodes //
    TURET, // [ ] Point defense (How to handle active tracking?)
    WEPON, // [ ] Attacks player, Controlled by turret, Always a leaf
    PIVOT, // [ ] Rotates subtree about local X (How to handle control by turret?)
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
    bool /*------*/ drawn; // -- Will this node be rendered?
    dynaPtr /*---*/ drawable; // `DynaMesh` that renders this node

    /// Constructor ///
    L_Node(){
        // Empty node
        type     = EMPTY;
        parent   = nullptr;
        Trel     = MatrixIdentity();
        Tabs     = MatrixIdentity();
        drawn    = false;
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
        if( drawn ){  
            drawable->xfrm = Tabs;  
            drawable->draw();
        }
        for( nodePtr& child : children ){  child->draw();  }
    }
};

///// Factories //////////////////////////////////

/*  +Z: Is always UP
     ^
     | ^ +Y: Positive lateral offset moves attachment point in this local direction
     |/
     *----> +X: Default growth direction, normal to parent module
*/

/*
> Modules are the rendered bodies (`drawable`s) of nodes
> Not all nodes have a module
> All nodes are on the surfaces of modules
> Modules do not need their own abstraction
> A single module of the building is drawn by one node, but has many undrawn nodes as attachment points
*/

nodePtr make_tower( nodePtr parent_,  float unit, 
                    int distance, int length, int height, int zOffset, int yOffset ){
    /* Towers always have a square `length` x `length` vertical cross section
       Non-zero `height` and `depth` towers are capped by regular and inverted turrets, respectively */

    // Init
    nodePtr rtnNode{ new L_Node{} };
    Vector3 dims = { length*unit, length*unit, height*unit };
    // Distal side pose, default growth direction AND +Y side pose
    Basis   sideA;
    Matrix  matxA;
    // Proximal side pose, facing parent AND -Y side pose
    Basis   sideB;
    Matrix  matxB;

    //// Build Internal Nodes ////
    // FIXME, START HERE: GRID THE SURFACE OF THE THE TOWER WITH NODES EXCEPT FOR THIS NODE
    if( (yOffset < 0) || (yOffset > length) ){
        cout << "Bad `yOffset` given, " << yOffset << " with length " << length << endl;
        return rtnNode;
    }
    // Get prototype poses for two sides
    // Distal side pose
    sideA = Basis::origin();
    matxA = sideA.get_homog();
    // Proximal side pose
    sideB = Basis::from_Xb_and_Zb( Vector3{ -1.0, 0.0, 0.0 }, Vector3{ 0.0, 0.0, 1.0 } );
    matxB = sideB.get_homog();

    // FIXME: THE Y OFFSET CAN BE HANDLED IN THE SAME OUTER LOOP, START COUNT NEGATIVE 
    // FIXME: THE Z OFFSET CAN BE HANDLED IN THE SAME INNER LOOP, START COUNT NEGATIVE 

    for( int i = -yOffset; i < length-yOffset; ++i ){
        for( int j = -zOffset; j < height-zOffset; ++j ){
            // Build distal side nodes, default growth direction
            // Build proximal side nodes, facing parent
        }
    }

    // Get prototype poses for two sides
    //
    // sideA = Basis::origin();
    // matxA = sideA.get_homog();
    // 
    // sideB = Basis::from_Xb_and_Zb( Vector3{ -1.0, 0.0, 0.0 }, Vector3{ 0.0, 0.0, 1.0 } );
    // matxB = sideB.get_homog();

    // FIXME: NODES FOR +/-Y SIDES

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
    cuboid.set_shader( lightShader.shader );

    Wedge  wedge{  2.0f, 2.0f, 2.0f, GREEN };
    wedge.set_posn( Vector3{ 0.0f, 0.0f, 1.25f } );
    wedge.set_shader( lightShader.shader );

    /// L-System ///
    L_Node root{};
    root.drawn = true;
    root.drawable = dynaPtr( new Cuboid{ 2.0f, 2.0f, 3.0f, BLUE  } );
    root.drawable->set_shader( lightShader.shader );

    nodePtr top{ new L_Node{} };
    top->drawn = true;
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
        root.children[0]->Trel = rotate_RPY_vehicle( root.children[0]->Trel, M_PI/60.0f, 0.0f, 0.0f );
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
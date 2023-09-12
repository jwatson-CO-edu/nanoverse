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

class L_Rule{ public:
    // Base class for L-System transformation rules
    nodePtr node; // ------- Node to be transformed
    virtual void run(){}; // Transformation strategy
};
typedef shared_ptr<L_Rule> rulePtr;

enum NodeType{
    EMPTY, // Default node, not rendered or updated
    TOWER, // Defensive structure, capped on top and bottom by turrets
    TURET, // Point defense
};

struct L_Port{
    // Holds a potential attachment point where an edge may pass through in 3D space
    Vector3 /*----*/ posn; // Position in the parent's frame
    Vector3 /*----*/ norm; // Vector pointing away from the parent's rendered body
    Vector3 /*----*/ upDr; // Global "up" vector, expressed in the parent's frame
    vector<NodeType> spec; // Constraints on attachment type
};

class L_Node{ public:
    // Basis for a graphical Lindenmayer System

    /// Members ///
    NodeType /*--*/ type; // --- String designator governing rendering and rules
    float /*-----*/ unitLen; //- Unit length [m] for generating geometry
    vvf /*-------*/ params; // - Generic node info for sharing, TBD
    vector<L_Port>  ports; // -- "Attachment points" potential edges may pass through
    nodePtr /*---*/ parent; // - Edge from parent
    vector<nodePtr> children; // Edges to children
    rulePtr /*---*/ rule; // --- Production rule for transforming the graph, FIXME: MANY INSTANCES OR ONE?
    Matrix /*----*/ Trel; // --- Relative transform from the parent frame
    Matrix /*----*/ Tabs; // --- Absolute transform in the world frame
    dynaPtr /*---*/ drawable; // `DynaMesh` that renders this node

    /// Constructor ///
    L_Node(){
        // Empty node
        type     = EMPTY;
        parent   = nullptr;
        rule     = nullptr;
        Trel     = MatrixIdentity();
        Tabs     = MatrixIdentity();
        drawable = nullptr;
    }

    /// Methods ///

    void add_child( nodePtr nuChild ){
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
        if( drawable ){  
            drawable->xfrm = Tabs;  
            drawable->draw();
        }
        for( nodePtr& child : children ){  child->draw();  }
    }
};

///// Factories //////////////////////////////////

nodePtr make_tower( nodePtr parent_, L_Port attachment, float unit, uint distance, uint height, uint depth ){
    /* Towers always have a square `unit` x `unit` vertical cross section
       Non-zero `height` and `depth` towers are capped by regular and inverted turrets, respectively */
    nodePtr rtnNode{ new L_Node{} };
    float   halfUnit = unit/2.0f;
    Vector3 nucleus  = Vector3Add( attachment.posn, Vector3Scale( attachment.norm, distance*unit ) );
    /* FIXME, START HERE: DO THE FOLLOWING 
        - Calc the center and dimensions of the `Cuboid`
        - Create `Cuboid`
        - Generate ports */
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
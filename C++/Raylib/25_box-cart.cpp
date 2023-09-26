// g++ 25_box-cart.cpp -std=c++17 -lraylib -O3 -o boxkart.out
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

struct XY_Grid{
    // Simplest X-Y grid with regular spacing

    /// Members ///
    Vector3 cntr;
    float   xLen;
    float   yLen;
    float   unit;
    Color   colr;

    /// Construction Constants ///
    float xMin;
    float xMax;
    float yMin;
    float yMax;

    /// Constructor(s) ///

    XY_Grid( const Vector3& cntr_, float xLen_, float yLen_, float unit_, Color colr_ ){
        // Set vars for drawing
        cntr = cntr_;
        xLen = xLen_;
        yLen = yLen_;
        unit = unit_;
        colr = colr_;
        xMin = cntr.x - xLen/2.0f;
        xMax = cntr.x + xLen/2.0f;
        yMin = cntr.y - yLen/2.0f;
        yMax = cntr.y + yLen/2.0f;
    }

    /// Methods ///

    void draw(){
        // Draw the grid using lines
        float   X = unit;
        float   Y = unit;
        rlBegin( RL_LINES );

        rlColor4ub( colr.r, colr.g, colr.b, colr.a );

        rlVertex3f( cntr.x, yMin, cntr.z );
        rlVertex3f( cntr.x, yMax, cntr.z );

        while( (cntr.x + X) <= xMax ){
            rlVertex3f( cntr.x + X, yMin, cntr.z );
            rlVertex3f( cntr.x + X, yMax, cntr.z );
            rlVertex3f( cntr.x - X, yMin, cntr.z );
            rlVertex3f( cntr.x - X, yMax, cntr.z );
            X += unit;
        }

        rlVertex3f( xMin, cntr.y, cntr.z );
        rlVertex3f( xMax, cntr.y, cntr.z );

        while( (cntr.y + Y) <= yMax ){
            rlVertex3f( xMin, cntr.y + Y, cntr.z );
            rlVertex3f( xMax, cntr.y + Y, cntr.z );
            rlVertex3f( xMin, cntr.y - Y, cntr.z );
            rlVertex3f( xMax, cntr.y - Y, cntr.z );
            Y += unit;
        }

        rlEnd();
    }
};  

////////// VEHICLES ////////////////////////////////////////////////////////////////////////////////


class CompositeModel{ public:
    // Contains multiple `DynaMesh` parts

    /// Members ///

    Matrix /*----*/ xfrm; // --- Pose of the entire model
    vector<dynaPtr> parts; // -- Drawable components
    Color /*-----*/ prtColor; // Main color of meshes
    // vector<segment> lines; // -- Drawable line segments
    // Color /*-----*/ linColor; // Color of line segments

    /// Constructor(s) ///

    CompositeModel(){
        // Default pose is the origin
        xfrm     = MatrixIdentity();
        prtColor = BLUE;
    }

    /// Methods ///

    void set_shader( Shader shader ){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->set_shader( shader );  }
    }

    void set_position( const Vector3& posn ){
        // Set the position of the `Cubeling`
        xfrm = set_posn( xfrm, posn );
    }

    void set_part_poses(){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->transform_from_parent( xfrm );  }
    }

    void draw(){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->draw();  }
    }

    size_t add_component( dynaPtr part ){
        // Add a component and return the current part count
        parts.push_back( part );
        return parts.size();
    }
};


class BoxKart : public CompositeModel { public:
    // A funky little cart with Katamari steering and (very) simple dynamics, +X is forward
    // Version 0.1: Planar movement only, 6 wheels, No slip, No air, Implied gravity only, No terrain/obstacle interaction
    
    /// Appearance ///
    float xLen;
    float yLen;
    float zLen;
    float wheelRad;
    float wheelHgt;

    /// Control ///
    float leftSteer;
    float leftEffort;
    float rghtSteer;
    float rghtEffort;

    /// Constructor(s) ///

    // FIXME, START HERE: WRITE CONSTRUCTOR, DO NOT DRAW WHEEL FRAMES FOR NOW

    BoxKart( float xLen_, float yLen_, float zLen_, float wheelRad_, Color bodyColor = BLUE ) : CompositeModel() { 
        // Basic cart geometry

        // 1. Set params
        xLen     = xLen_;
        yLen     = yLen_;
        zLen     = zLen_;
        wheelRad = wheelRad_;
        // wheelHgt = wheelHgt_;
        prtColor = bodyColor;

        float axelZ = -0.25f * (zLen/2.0f + 2.0f*wheelRad);
        float axelY = yLen / 2.0f;

        // 2. Create body
        dynaPtr nuPart = dynaPtr( new Cuboid{ xLen, yLen, zLen, prtColor } );
        parts.push_back( nuPart );

        // 3. Create left wheels
        Matrix T = set_posn( MatrixRotateX( M_PI/2.0f ), Vector3{ 0.0f, -axelY, axelZ } );
        
        // Back Left
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, prtColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ -xLen/4.0f, 0.0f, 0.0f } );
        parts.push_back( nuPart );

        // Middle Left
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, prtColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ 0.0f, 0.0f, 0.0f } );
        parts.push_back( nuPart );

        // Front Left
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, prtColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ xLen/4.0f, 0.0f, 0.0f } );
        parts.push_back( nuPart );

        // 3. Create right wheels
        T = set_posn( MatrixRotateX( -M_PI/2.0f ), Vector3{ 0.0f, axelY, axelZ } );
        
        // Back Right
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, prtColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ -xLen/4.0f, 0.0f, 0.0f } );
        parts.push_back( nuPart );

        // Middle Right
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, prtColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ 0.0f, 0.0f, 0.0f } );
        parts.push_back( nuPart );

        // Front Right
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, prtColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ xLen/4.0f, 0.0f, 0.0f } );
        parts.push_back( nuPart );

    };
};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Box Cart, Ver. 0.1" );
    SetTargetFPS( 60 );

    ///// Create Objects //////////////////////////////////////////////////

    /// Create Camera ///
    Camera camera = Camera{
        Vector3{  10.0,  10.0,  10.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    /// Lighting ///
    Lighting lightShader{};
    lightShader.set_camera_posn( camera );

    XY_Grid xyGrid{ Vector3Zero(), 20.0f, 20.0f, 1.0f, RAYWHITE };

    BoxKart kart{ 3.0f, 2.0f, 1.0f, 0.5f, GREEN };
    kart.set_shader( lightShader.shader );

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        lightShader.update();

        xyGrid.draw();
        kart.set_part_poses();
        kart.draw();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}
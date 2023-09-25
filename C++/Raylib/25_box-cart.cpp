// g++ 25_box-cart.cpp -std=c++17 -lraylib -O3
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

class CompositeModel{ public:
    // Contains multiple `DynaMesh` parts

    /// Members ///

    Matrix /*----*/ xfrm;
    vector<dynaPtr> parts;

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
};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Cubelings" );
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

    XY_Grid xyGrid{ Vector3Zero(), 20.0f, 20.0f, 1.0f, RAYWHITE };

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        xyGrid.draw();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}
// g++ 27_box-kart-02.cpp -std=c++17 -lraylib -O3 -o boxkart.out
// Arcade kart with Katamari steering


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////////////////

template <typename T> T sgn(T val){
    // Return the sign of the number, or zero for zero magnitude
    // Original Author: Stef, https://stackoverflow.com/a/4609795
    return (T) ((T(0) < val) - (val < T(0)));
}

float Vector2CrossProduct( const Vector2& op1, const Vector2& op2 ){
    // Return the magnitude of the cross product of two 2D vectors
    return ( op1.x * op2.y - op1.y * op2.x );
}

Matrix translate_XY( const Matrix& xfrm, const Vector2& trns ){
    // Move the pose within the local XY plane

    // FIXME: NEED TO ROUND THE TINY VALUES TO ZERO

    return translate( xfrm, Vector3Add(
        Vector3Transform( {trns.x, 0.0f  , 0.0f}, xfrm ),
        Vector3Transform( {0.0f  , trns.y, 0.0f}, xfrm )
    ) );
}

ostream& operator<<( ostream& os , const Vector3& vec ) { 
    // ostream '<<' operator for Raylib Color
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "{X: "  << ((float) vec.x);
    os << ", Y: " << ((float) vec.y);
    os << ", Z: " << ((float) vec.z);
    os << "}";
    return os; // You must return a reference to the stream!
}

ostream& operator<<( ostream& os , const Vector2& vec ) { 
    // ostream '<<' operator for Raylib Color
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "{X: "  << ((float) vec.x);
    os << ", Y: " << ((float) vec.y);
    os << "}";
    return os; // You must return a reference to the stream!
}

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

    Vector3 get_position(){
        // Set the position of the `Cubeling`
        return get_posn( xfrm );
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

void add_stripes_to_Cylinder( dynaPtr cylinder, Color color1, Color color2 ){
    // Color rectangular faces with alternating colors
    // NOTE: This function assumes that there is an even number of rectangles
    Color iterColor;
    cylinder->wipe_geo( false, true );
    for( uint i = 0; i < (cylinder->Ntri)/4; ++i ){
        if( (i%2)==0 ){  iterColor = color1;  }else{  iterColor = color2;  }
        for( ubyte j = 0; j < 4; ++j ){
            cylinder->clrs.push_back( {iterColor, iterColor, iterColor} );
        }
    }
    cylinder->load_mesh_buffers( false, true );
}

float dist_to_square_edge( float angle_rad, float halfLen = 1.0f ){
    // Return the distance from the center of the square (with lengths `halfLen*2`) to the edge pointed to by `angle_rad`
    float angle = abs( fmod( angle_rad, M_PI ) );
    float x, y;
    if( (angle == 0.0) || (angle == M_PI) ){
        x = halfLen;
        y = 0.0f;
    }else if( angle <= (M_PI/4.0f) ){
        x = halfLen;
        y = x * tan( angle );
    }else if( angle < (M_PI/2.0f) ){
        angle = (M_PI/2.0f) - angle;
        y     = halfLen;
        x     = y * tan( angle );
    }else if( angle == (M_PI/2.0f) ){
        y     = halfLen;
        x     = 0.0f;
    }else if( angle <= (3.0f*M_PI/4.0f) ){
        angle -= M_PI/2.0f; 
        y     = halfLen;
        x     = y * tan( angle );
    }else{
        angle = M_PI - angle;
        x     = halfLen;
        y     = x * tan( angle );
    }
    return sqrtf( x*x + y*y );
}

class BoxKart : public CompositeModel { public:
    // A funky little cart with Katamari steering and (very) simple dynamics, +X is forward
    // Version 0.1: Planar movement only, 6 wheels, No slip, No air, Implied gravity only, No terrain/obstacle interaction
    
    /// Appearance ///
    float xLen;
    float yLen;
    float zLen;
    float wheelRad;
    float wheelHgt;
    float leftWhlTheta;
    float rghtWhlTheta;

    /// Control ///
    float turnMax;
    float driveMax;

    /// Constructor(s) ///

    BoxKart( float xLen_, float yLen_, float zLen_, float wheelRad_, Color bodyColor = BLUE ) : CompositeModel() { 
        // Basic cart geometry

        // 1. Set appearance params
        xLen     = xLen_;
        yLen     = yLen_;
        zLen     = zLen_;
        wheelRad = wheelRad_;
        prtColor = bodyColor;
        
        leftWhlTheta = 0.0f;
        rghtWhlTheta = 0.0f;

        float axelZ    = -1.0f * (zLen/2.0f + 1.5f*wheelRad);
        float axelY    = yLen / 2.0f;
        Color whlColor = GRAY;

        // 2. Set control params
        turnMax  = 1.5f * M_PI / 360.0f;
        driveMax = 0.065;

        // 3. Create body
        dynaPtr nuPart = dynaPtr( new Cuboid{ xLen, yLen, zLen, prtColor } );
        parts.push_back( nuPart );

        // 4. Create left wheels
        Matrix T = set_posn( MatrixRotateX( M_PI/2.0f ), Vector3{ 0.0f, -axelY, axelZ } );
        
        // Back Left
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, whlColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ -xLen/2.0f, 0.0f, 0.0f } );
        add_stripes_to_Cylinder( nuPart, GRAY, DARKGRAY );
        parts.push_back( nuPart );

        // Middle Left
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, whlColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ 0.0f, 0.0f, 0.0f } );
        add_stripes_to_Cylinder( nuPart, GRAY, DARKGRAY );
        parts.push_back( nuPart );

        // Front Left
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, whlColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ xLen/2.0f, 0.0f, 0.0f } );
        add_stripes_to_Cylinder( nuPart, GRAY, DARKGRAY );
        parts.push_back( nuPart );

        // 5. Create right wheels
        T = set_posn( MatrixRotateX( -M_PI/2.0f ), Vector3{ 0.0f, axelY, axelZ } );
        
        // Back Right
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, whlColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ -xLen/2.0f, 0.0f, 0.0f } );
        add_stripes_to_Cylinder( nuPart, GRAY, DARKGRAY );
        parts.push_back( nuPart );

        // Middle Right
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, whlColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ 0.0f, 0.0f, 0.0f } );
        add_stripes_to_Cylinder( nuPart, GRAY, DARKGRAY );
        parts.push_back( nuPart );

        // Front Right
        nuPart = dynaPtr( new Cylinder{ wheelRad, wheelRad, whlColor } );
        nuPart->Trel = T;
        nuPart->Tcur = set_posn( MatrixIdentity(), Vector3{ xLen/2.0f, 0.0f, 0.0f } );
        add_stripes_to_Cylinder( nuPart, GRAY, DARKGRAY );
        parts.push_back( nuPart );
    }

    void move_forward( float dX ){
        // Drive in the local +X direction
        xfrm = move_X_vehicle( xfrm, dX );
    }

    void move_XY( const Vector2& trns ){
        // Drive in the XY plane direction
        if( Vector2LengthSqr( trns ) > 0.0f )  xfrm = translate_XY( xfrm, trns );
    }

    void turn( float theta ){
        // Positive turn about the local +Z
        xfrm = MatrixMultiply( MatrixRotateZ( theta ), xfrm );
    }

    void control_law( float leftStickX, float leftStickY, float rghtStickX, float rghtStickY ){
        // Adjust speed and orientation of kart and wheels according to input
        // This function assumes input is in [-1,+1] --is-> [Back,Front]
        float rotTheta;

        float axelZ = -1.0f * (zLen/2.0f + 1.5f*wheelRad);
        float axelY = yLen / 2.0f;
        float axelX = xLen / 2.0f;

        // Circle the Square: Treat the thumbsticks as though their output is confined to a unit circle

        float   leftTheta = atan2f( leftStickX, leftStickY );
        float   leftMag   = sqrtf( leftStickX*leftStickX + leftStickY*leftStickY ) / dist_to_square_edge( leftTheta );
        Vector2 leftVec   = { leftMag * cos( leftTheta ), leftMag * sin( leftTheta ) };

        float   rghtTheta = atan2f( rghtStickX, rghtStickY );
        float   rghtMag   = sqrtf( rghtStickX*rghtStickX + rghtStickY*rghtStickY ) / dist_to_square_edge( rghtTheta );
        Vector2 rghtVec   = { rghtMag * cos( rghtTheta ), rghtMag * sin( rghtTheta ) };

        Vector2 result  = Vector2Scale( Vector2Add( leftVec, rghtVec ), 0.5 );
        Vector2 resMove = Vector2Scale( result, driveMax );
        float   resTurn = (Vector2CrossProduct( leftVec, {0.0f, -1.0f} ) + Vector2CrossProduct( rghtVec, {0.0f, 1.0f} )) 
                          * 0.5 * turnMax;
        
        // move_forward(  (leftStickY + rghtStickY)/2.0f * driveMax  );
        // turn( /*----*/ (leftStickY - rghtStickY)/2.0f * turnMax   );

        move_XY( resMove );
        turn(    resTurn );

        // cout << "Resultant: " << result << endl;
        cout << "Displacement: " << resMove << ", Position: " << get_posn( xfrm ) << endl;
        // cout << dist_to_square_edge( leftTheta ) << ", " << dist_to_square_edge( rghtTheta ) << endl;
        // cout << resMove << ", " << resTurn << ", " << get_posn( xfrm ) << endl;
        // cout << get_posn( xfrm ) << endl;
        // cout << "Left Theta: ___ " << leftTheta << ", Right Theta: ___ " << rghtTheta << endl;
        // cout << "Left Magnitude: " << leftMag   << ", Right Magnitude: " << rghtMag << endl;
        // cout << "Left Vector: " << leftVec << ", Right Vector: " << rghtVec << endl;
        // cout << "Left Stick: "    << leftStickX << ", " << leftStickY 
        //      << ", Right Stick: " << rghtStickX << ", " << rghtStickY << endl;

        

        if( abs( leftTheta ) < (M_PI/2.0f) ){
            // leftWhlTheta -=  leftStickY * driveMax / (1.0f * wheelRad * M_PI);
            leftWhlTheta -=  leftMag * driveMax / (1.0f * wheelRad * M_PI);
            // leftTheta += (M_PI/2.0f) * sgn( leftTheta );
        }else{
            // leftWhlTheta +=  leftStickY * driveMax / (1.0f * wheelRad * M_PI);
            leftWhlTheta +=  leftMag * driveMax / (1.0f * wheelRad * M_PI);
        }  

        
        if( abs( rghtTheta ) < (M_PI/2.0f) ){
            // rghtWhlTheta -=  rghtStickY * driveMax / (1.0f * wheelRad * M_PI);
            rghtWhlTheta -=  rghtMag * driveMax / (1.0f * wheelRad * M_PI);
            // rghtTheta += (M_PI/2.0f) * sgn( rghtTheta );
        }else{
            // rghtWhlTheta +=  rghtStickY * driveMax / (1.0f * wheelRad * M_PI);
            rghtWhlTheta +=  rghtMag * driveMax / (1.0f * wheelRad * M_PI);
        }  

        // Set poses for Left Wheels
        for( ubyte i = 1; i < 4; ++i ){
            parts[i]->Trel = set_posn( MatrixRotateX( M_PI/2.0f ), Vector3{ (i*1.0f-2.0f)*axelX, -axelY, axelZ } );
            parts[i]->Tcur = MatrixMultiply( MatrixRotateZ( leftWhlTheta ), MatrixRotateY( leftTheta ) );
        }

        // Set poses for Right Wheels
        for( ubyte i = 4; i < 7; ++i ){
            parts[i]->Trel = set_posn( MatrixRotateX( M_PI/2.0f ), Vector3{ (i*1.0f-5.0f)*axelX, axelY, axelZ } );
            parts[i]->Tcur = MatrixMultiply( MatrixRotateZ( rghtWhlTheta ), MatrixRotateY( rghtTheta ) );
        }

        set_part_poses();
    }

};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Box Cart, Ver. 0.2" );
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

    XY_Grid xyGrid{ Vector3Zero(), 75.0f, 75.0f, 1.0f, RAYWHITE };

    BoxKart kart{ 3.0f, 2.0f, 1.0f, 0.5f, GREEN };
    kart.set_shader( lightShader.shader );
    kart.set_position( Vector3{ -10.0, 0.0, 2.0 } );

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        lightShader.update();

        // gamepad input
		if( IsGamepadAvailable(0) ){
            kart.control_law(
                GetGamepadAxisMovement( 0, GAMEPAD_AXIS_LEFT_X ),
                GetGamepadAxisMovement( 0, GAMEPAD_AXIS_LEFT_Y ),
                GetGamepadAxisMovement( 0, GAMEPAD_AXIS_RIGHT_X ),
                GetGamepadAxisMovement( 0, GAMEPAD_AXIS_RIGHT_Y )
            );
		}else{
            kart.control_law( 0.0f, 0.0f, 0.0f, 0.0f );
        }

        xyGrid.draw();
        kart.draw();
        

        camera.target = kart.get_position();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}
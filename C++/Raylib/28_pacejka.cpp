// g++ 28_pacejka.cpp -std=c++17 -lraylib -O3 -o boxkart.out
// Implement the simplest tire slip model


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// DYNAMICS ////////////////////////////////////////////////////////////////////////////////

struct PacejkaWheel{
    // Simplest frictional wheel model
    // https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/

    ///// Longitudinal Force //////////////////////////////////////////////

    float b00; // Shape factor 		1.4 .. 1.8 	1.5
    float b01; // Load influence on longitudinal friction coefficient (*1000) 	1/kN 	-80 .. +80 	0
    float b02; // Longitudinal friction coefficient (*1000) 		900 .. 1700 	1100
    float b03; // Curvature factor of stiffness/load 	N/%/kN^2 	-20 .. +20 	0
    float b04; // Change of stiffness with slip 	N/% 	100 .. 500 	300
    float b05; // Change of progressivity of stiffness/load 	1/kN 	-1 .. +1 	0
    float b06; // Curvature change with load^2 		-0.1 .. +0.1 	0
    float b07; // Curvature change with load 		-1 .. +1 	0
    float b08; // Curvature factor 		-20 .. +1 	-2
    float b09; // Load influence on horizontal shift 	%/kN 	-1 .. +1 	0
    float b10; // Horizontal shift 	% 	-5 .. +5 	0
    float b11; // Vertical shift 	N 	-100 .. +100 	0
    float b12; // Vertical shift at load = 0 	N 	-10 .. +10 	0
    float b13; // Curvature shift
    float slip; // Slip ratio as a percentage

    ///// Lateral Force ///////////////////////////////////////////////////
    
    float a00; // Shape factor 		1.2 .. 18 	1.4
    float a01; // Load influence on lateral friction coefficient (*1000) 	1/kN 	-80 .. +80 	0
    float a02; // Lateral friction coefficient (*1000) 		900 .. 1700 	1100
    float a03; // Change of stiffness with slip 	N/deg 	500 .. 2000 	1100
    float a04; // Change of progressivity of stiffness / load 	1/kN 	0 .. 50 	10
    float a05; // Camber influence on stiffness 	%/deg/100 	-0.1 .. +0.1 	0
    float a06; // Curvature change with load 		-2 .. +2 	0
    float a07; // Curvature factor 		-20 .. +1 	-2
    float a08; // Load influence on horizontal shift 	deg/kN 	-1 .. +1 	0
    float a09; // Horizontal shift at load = 0 and camber = 0 	deg 	-1 .. +1 	0
    float a10; // Camber influence on horizontal shift 	deg/deg 	-0.1 .. +0.1 	0
    float a11; // Vertical shift 	N 	-200 .. +200 	0
    float a12; // Vertical shift at load = 0 	N 	-10 .. +10 	0
    float a13; // Camber influence on vertical shift, load dependent 	N/deg/kN 	-10 .. +10 	0
    float a14; // Camber influence on vertical shift 	N/deg 	-15 .. +15 	0
    float a15; // Camber influence on lateral friction coefficient 	1/deg 	-0.01 .. +0.01 	0
    float a16; // Curvature change with camber 		-0.1 .. +0.1 	0
    float a17; // Curvature shift 		-1 .. +1 	0

    ///// Calculations ////////////////////////////////////////////////////

    float F_longitudinal( float Fz ){
        float C   = b00;
        float D   = Fz * (b01 * Fz + b02);
        float BCD = (b03*Fz*Fz + b04*Fz) * expf(-b05 * Fz);
        float B   = BCD / (C * D);
        float H   = b09*Fz + b10;
        float V   = b11*Fz + b12;
        float E   = (b06*Fz*Fz + b07*Fz + b08) * (1 - b13*sgn(slip + H));
        float Bx1 = B * (slip + H);
        return D * sinf(C * atanf(Bx1 - E * (Bx1 - atanf(Bx1)))) + V;
    }
};


////////// VEHICLES ////////////////////////////////////////////////////////////////////////////////

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
        float   resTurn = (Vector2CrossProduct( leftVec, {0.0f, 1.0f} ) + Vector2CrossProduct( rghtVec, {0.0f, -1.0f} )) 
                          * 0.5 * turnMax;

        move_XY( resMove );
        turn(    resTurn );

        leftWhlTheta -=  leftMag * driveMax / (1.0f * wheelRad * M_PI);
        rghtWhlTheta -=  rghtMag * driveMax / (1.0f * wheelRad * M_PI);

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
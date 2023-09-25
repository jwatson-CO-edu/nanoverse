// g++ 24_cubelings.cpp -std=c++17 -lraylib -O3
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

enum C_State{
    TURN, // Change orientation
    ZFLY,
    REST,
};

class Cubeling{ public: 
    // Cubic creature with oscillating pegs on each face

    /// Members ///

    Matrix /*----*/ xfrm;
    dynaPtr /*---*/ body;
    vector<dynaPtr> pegs;
    vvec3 /*-----*/ posns;
    float /*-----*/ t;
    float /*-----*/ f;
    float /*-----*/ A;
    C_State /*---*/ state;
    uint /*------*/ timer, tAccl;
    float dR, dP, dY, dZ, ddZ, zMax;
    
    /// Constructor ///

    Cubeling( float edgeLen, float pegDia, float pegLen, Color color ){
        // Create the parts of the creature and set poses

        state = REST;
        timer = 0;
        zMax  = 0.0125f;
        tAccl = 10;
        ddZ   = zMax/(1.0f*tAccl);
        dZ    = 0.0f; 

        Basis  relaB;
        Matrix relaM;
        Matrix currM;
        posns = { {-edgeLen/8.0f, -edgeLen/8.0f, edgeLen/4.0f} , 
                  {-edgeLen/8.0f,  edgeLen/8.0f, edgeLen/4.0f} ,
                  { edgeLen/8.0f, -edgeLen/8.0f, edgeLen/4.0f} ,
                  { edgeLen/8.0f,  edgeLen/8.0f, edgeLen/4.0f} };
        
        // 1. Init geo
        body = dynaPtr( new Cube{ edgeLen, color } );
        for( ubyte i = 0; i < 24; ++i ){
            pegs.push_back( dynaPtr( new Cylinder{ pegDia/2.0f, pegLen, color } ) );
        }

        // 2. Init oscillations
        t = 0.0f;
        f = 2.0f;
        A = pegLen*0.9f/4.0f;

        // 3. Init poses
        xfrm = MatrixIdentity();
        
        // Face 1 //
        relaB = Basis::from_Xb_and_Zb( Vector3{0.0f,0.0f,1.0}, Vector3{1.0f,0.0f,0.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 0; i < 4; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }

        // Face 2 //
        relaB = Basis::from_Xb_and_Zb( Vector3{0.0f,0.0f,1.0}, Vector3{0.0f,1.0f,0.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 4; i < 8; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }

        // Face 3 //
        relaB = Basis::from_Xb_and_Zb( Vector3{0.0f,0.0f,1.0}, Vector3{-1.0f,0.0f,0.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 8; i < 12; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }

        // Face 4 //
        relaB = Basis::from_Xb_and_Zb( Vector3{0.0f,0.0f,1.0}, Vector3{0.0f,-1.0f,0.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 12; i < 16; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }

        // Face 5 //
        relaB = Basis::from_Xb_and_Zb( Vector3{1.0f,0.0f,0.0}, Vector3{0.0f,0.0f,1.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 16; i < 20; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }

        // Face 6 //
        relaB = Basis::from_Xb_and_Zb( Vector3{1.0f,0.0f,0.0}, Vector3{0.0f,0.0f,-1.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 20; i < 24; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }
    }

    /// Methods ///

    void set_shader( Shader shader ){
        // Set the shader for all parts
        body->set_shader( shader );
        for( dynaPtr& peg : pegs ){
            peg->set_shader( shader );
        }
    }

    void set_position( const Vector3& posn ){
        // Set the position of the `Cubeling`
        xfrm = set_posn( xfrm, posn );
    }

    void transition(){
        // Update state
        switch( state ){
            case REST:
                if( timer == 0 ){
                    state = TURN;
                    timer = 100;
                    dR    = randf( -M_PI, M_PI ) / (timer - 1.0f);
                    dP    = randf( -M_PI, M_PI ) / (timer - 1.0f);
                    dY    = randf( -M_PI, M_PI ) / (timer - 1.0f);
                }else{
                    --timer;
                }
                break;

            case TURN:
                if( timer == 0 ){
                    state = ZFLY;
                    timer = 120;
                }else{
                    --timer;
                    xfrm = rotate_RPY_vehicle( xfrm, dR, dP, dY );
                }
                break;

            case ZFLY:
                if( timer == 0 ){
                    state = REST;
                    timer = 30;
                }else{
                    --timer;
                    if( dZ < zMax ) dZ += ddZ;
                    if( timer < tAccl ) dZ -= ddZ;
                    xfrm = thrust_Z_vehicle( xfrm, dZ );
                    // xfrm = translate( xfrm, Vector3{0.0,0.0,dZ} );
                }
                break;
            
            default:
                cout << "BAD STATE" << endl;
                break;
        }
    }

    void update(){
        // Update poses for all parts
        transition();
        body->xfrm = xfrm;
        t += 0.01;
        for( ubyte i = 0; i < 24; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Tcur = set_posn( MatrixIdentity(), Vector3Add(
                posns[i%4],
                Vector3{0.0f, 0.0f, A*sinf( t*f + (i%4)*M_PI/2.0f )}
            ) );
            pegs[i]->transform_from_parent( xfrm );
        }
    }

    void draw(){
        body->draw();
        for( dynaPtr& peg : pegs ){
            peg->draw();
        }
    }
};
typedef shared_ptr<Cubeling> cblnPtr;



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

    /// Lighting ///
    Lighting lightShader{};
    lightShader.set_camera_posn( camera );

    // Cube cube( 1.0, DARKBLUE );
    // cube.set_shader( lightShader.shader );

    // Cylinder cyl{ 1.0, 1.0, DARKBLUE };
    // cyl.set_shader( lightShader.shader );

    Cubeling animal{ 1.0, 0.25, 0.5, DARKBLUE };
    animal.set_shader( lightShader.shader );

    vector<cblnPtr> creatures;
    cblnPtr nuCube;
    for( uint i = 0; i < 100; ++i ){
        nuCube = cblnPtr( new Cubeling{ 1.0, 0.25, 0.5, uniform_random_color() } );
        nuCube->set_position( uniform_vector_noise( 5.0f ) );
        nuCube->set_shader( lightShader.shader );
        creatures.push_back( nuCube ); 
    }
    

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        lightShader.update();
        // cube.draw();
        // cyl.draw();
        // animal.update();
        // animal.draw();

        for( cblnPtr& creature : creatures ){
            creature->update();
            creature->draw();
        }

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}

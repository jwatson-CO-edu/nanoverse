// g++ 24_cubelings.cpp -std=c++17 -lraylib -O3
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



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
        load_mesh_buffers( true, true );
    }
};

class Cylinder : public DynaMesh{ public:
    // Simple Cylinder

    /// Constructors ///

    Cylinder( float radius, float length, Color color, uint segments = 16 ) : DynaMesh( segments*4 ){
        // Build and save geo
        
        // 0. Init
        triPnts pushTri;
        triClrs pushClr;
        Vector3 norm, p1, p2, p3, p4, n1, n2;
        vvec3   V;
        float   stepTurn = 2.0f*M_PI/(1.0f*segments);
        float   theta;
        float   halfLen = length/2.0;
        Vector3 c1 = Vector3{ 0.0f, 0.0f,  halfLen };
        Vector3 c2 = Vector3{ 0.0f, 0.0f, -halfLen };
        uint    ip1;


        // 1. Establish vertices
        for( uint i = 1; i <= segments; ++i ){
            theta = 1.0f * i * stepTurn;
            V.push_back( Vector3{ radius*cos( theta ), radius*sin( theta ),  halfLen } );
            V.push_back( Vector3{ radius*cos( theta ), radius*sin( theta ), -halfLen } );
        }

        // 2. Build tris
        for( uint i = 0; i < segments; ++i ){
            ip1 = (i+1)%segments;
            p1 = V[i*2  ];  p3 = V[ip1*2  ];
            p2 = V[i*2+1];  p4 = V[ip1*2+1];
            n1 = Vector3Normalize( Vector3Subtract( p1, c1 ) );
            n2 = Vector3Normalize( Vector3Subtract( p3, c1 ) );

            tris.push_back( {p3, p1, p2} );
            nrms.push_back( {n2, n1, n1} );
            
            tris.push_back( {p3, p2, p4} );
            nrms.push_back( {n2, n1, n2} );

            n1 = Vector3{ 0.0f, 0.0f,  1.0f };
            tris.push_back( {p1, p3, c1} );
            nrms.push_back( {n1, n1, n1} );

            n2 = Vector3{ 0.0f, 0.0f, -1.0f };
            tris.push_back( {p4, p2, c2} );
            nrms.push_back( {n2, n2, n2} );
        }

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};

class Cubeling{ public: 
    // Cubic creature with oscillating pegs on each face
    Matrix /*----*/ xfrm;
    dynaPtr /*---*/ body;
    vector<dynaPtr> pegs;
    vvec3 /*-----*/ posns;
    float /*-----*/ t;
    float /*-----*/ f;
    float /*-----*/ A;
    

    Cubeling( float edgeLen, float pegDia, float pegLen, Color color ){
        // Create the parts of the creature and set poses

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

        // Face 5 //
        relaB = Basis::from_Xb_and_Zb( Vector3{1.0f,0.0f,0.0}, Vector3{0.0f,0.0f,-1.0} );
        relaM = relaB.get_homog();
        for( ubyte i = 20; i < 24; ++i ){
            // relaM = set_posn( relaM, posns[i%4] );
            pegs[i]->Trel = relaM; 
            pegs[i]->Tcur = set_posn( MatrixIdentity(), posns[i%4] );
        }
    }

    void set_shader( Shader shader ){
        body->set_shader( shader );
        for( dynaPtr& peg : pegs ){
            peg->set_shader( shader );
        }
    }

    void update(){
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

// FIXME: CREATE A POPULATION OF WANDERING CUBELINGS WITH SIMPLE BEHAVIOR AND DISPLAY 



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
        Vector3{   2.0,   2.0,   2.0 }, // Position
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
        animal.update();
        animal.draw();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}

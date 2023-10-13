// g++ 30_star-map.cpp -std=c++17 -lraylib -O3 -o starMap.out
// Animated star-map with bump-maps, Inspired by the Ahsoka end-credits sequence


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// HELPERS /////////////////////////////////////////////////////////////////////////////////

Vector3 vec3d_from_arbitrary_2D_basis( float x, float y, const Vector3& xBasis, const Vector3& yBasis ){
    // Return a coordinate in an arbitrary (non-orthoginal) 2D basis nested within a 3D frame
    // DO NOT normalize the basis vectors , see below!
	return Vector3Add(  Vector3Scale( xBasis, x ),  Vector3Scale( yBasis, y )  ); 
}

triPnts flip_tri_outward( const triPnts& tri ){
    // Point the triangle normal away from the origin
    if( Vector3DotProduct( normal_of_tiangle( tri ), tri[0] ) > 0.0f )  return tri;
    return {tri[0], tri[2], tri[1]};
}

////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

class Icosahedron : public DynaMesh { public:
    // Good ol' Platonic Solid

    /// Members ///

    // ~ Constants ~
	const float sqrt5 = sqrt( 5.0f ); // ----------------------------------- Square root of 5
	const float phi   = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------- The Golden Ratio
	const float ratio = sqrt( 10.0f + ( 2.0f * sqrt5 ) ) / ( 4.0f * phi ); // ratio of edge length to radius
	
	// ~ Variables ~
	float radius;
	float a; 
	float b; 
    vvec3 V;

    // ~ Appearance ~
    Color baseClr;
    bool  anim;
    float rolVel;
    float ptcVel;
    float yawVel;

    Icosahedron( float rad , const Vector3& cntr, Color color = BLUE, bool active = true ) : DynaMesh( 20 ){
        // Compute the vertices and faces
        // NOTE: This is a building block for the stellated sphere

        // ~ Geometry Pre-Computation ~
        set_posn( cntr );
        radius = rad;
        // colr   = color;
        a /**/ = ( radius / ratio ) * 0.5;
        b /**/ = ( radius / ratio ) / ( 2.0f * phi );

        // ~ Animation ~
        anim = active;
        float loRate = -0.01f;
        float hiRate =  0.01f;
        rolVel = randf( loRate, hiRate );
        ptcVel = randf( loRate, hiRate );
        yawVel = randf( loRate, hiRate );

        // Define the icosahedron's 12 vertices:
        V.push_back( Vector3{  0,  b, -a } );
        V.push_back( Vector3{  b,  a,  0 } );
        V.push_back( Vector3{ -b,  a,  0 } );
        V.push_back( Vector3{  0,  b,  a } );
        V.push_back( Vector3{  0, -b,  a } );
        V.push_back( Vector3{ -a,  0,  b } );
        V.push_back( Vector3{  0, -b, -a } );
        V.push_back( Vector3{  a,  0, -b } );
        V.push_back( Vector3{  a,  0,  b } );
        V.push_back( Vector3{ -a,  0, -b } );
        V.push_back( Vector3{  b, -a,  0 } );
        V.push_back( Vector3{ -b, -a,  0 } );

        // Define the icosahedron's 20 triangular faces: CCW-out
        push_triangle_w_norms( {V[ 2], V[ 1], V[ 0]} );
        push_triangle_w_norms( {V[ 1], V[ 2], V[ 3]} );
        push_triangle_w_norms( {V[ 5], V[ 4], V[ 3]} );
        push_triangle_w_norms( {V[ 4], V[ 8], V[ 3]} );
        push_triangle_w_norms( {V[ 7], V[ 6], V[ 0]} );
        push_triangle_w_norms( {V[ 6], V[ 9], V[ 0]} );
        push_triangle_w_norms( {V[11], V[10], V[ 4]} );
        push_triangle_w_norms( {V[10], V[11], V[ 6]} );
        push_triangle_w_norms( {V[ 9], V[ 5], V[ 2]} );
        push_triangle_w_norms( {V[ 5], V[ 9], V[11]} );
        push_triangle_w_norms( {V[ 8], V[ 7], V[ 1]} );
        push_triangle_w_norms( {V[ 7], V[ 8], V[10]} );
        push_triangle_w_norms( {V[ 2], V[ 5], V[ 3]} );
        push_triangle_w_norms( {V[ 8], V[ 1], V[ 3]} );
        push_triangle_w_norms( {V[ 9], V[ 2], V[ 0]} );
        push_triangle_w_norms( {V[ 1], V[ 7], V[ 0]} );
        push_triangle_w_norms( {V[11], V[ 9], V[ 6]} );
        push_triangle_w_norms( {V[ 7], V[10], V[ 6]} );
        push_triangle_w_norms( {V[ 5], V[11], V[ 4]} );
        push_triangle_w_norms( {V[10], V[ 8], V[ 4]} );

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }

    void update(){  rotate_RPY( rolVel, ptcVel, yawVel );  } // Rotate
};


class Sphere : public DynaMesh { public:
    // A sphere constructed from a subdivided icosahedron in order to create facets of near-equal area (Good for sims?) 

    Sphere( float rad , const Vector3& cntr, ubyte div = 3, Color color = BLUE ) : 
            DynaMesh( 20 * (div*(div+1)/2 + (div-1)*(div)/2) ) {
        Icosahedron icos{ rad, cntr };
        Vector3 v0, v1, v2, xTri, yTri, temp, vA, vB, vC, nA, nB, nC;
        for( triPnts& tri : icos.tris ){
            v0 = tri[0];  v1 = tri[1];  v2 = tri[2];
            xTri = Vector3Scale( Vector3Subtract( v1, v0 ), 1.0f/div );
            yTri = Vector3Scale( Vector3Subtract( v2, v0 ), 1.0f/div );

            for( ubyte row = 1; row <= div; ++row ){
                for( ubyte j = row ; j > 0 ; j-- ){ // Construct the v0-pointing tris
                    vA = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j  ), (float) (row-j  ), xTri, yTri ) );
                    vB = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j-1), (float) (row-j+1), xTri, yTri ) );
                    vC = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j-1), (float) (row-j  ), xTri, yTri ) );
                    nA = Vector3Normalize( vA );
                    nB = Vector3Normalize( vB );
                    nC = Vector3Normalize( vC );
                    vA = Vector3Scale( nA, rad );
                    vB = Vector3Scale( nB, rad );
                    vC = Vector3Scale( nC, rad );
                    tris.push_back( {vA, vB, vC} );
                    nrms.push_back( {nA, nB, nC} );
                }
                for( ubyte j = row - 1 ; j > 0 ; j-- ){ // Construct the anti-v0-pointing tris
                    vA = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j  ), (float) (row-1-j  ), xTri, yTri ) );
                    vB = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j  ), (float) (row-1-j+1), xTri, yTri ) );
                    vC = Vector3Add( v0, vec3d_from_arbitrary_2D_basis( (float) (j-1), (float) (row-1-j+1), xTri, yTri ) );
                    nA = Vector3Normalize( vA );
                    nB = Vector3Normalize( vB );
                    nC = Vector3Normalize( vC );
                    vA = Vector3Scale( nA, rad );
                    vB = Vector3Scale( nB, rad );
                    vC = Vector3Scale( nC, rad );
                    tris.push_back( {vA, vB, vC} );
                    nrms.push_back( {nA, nB, nC} );
                }
            }
        }

        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};


class EllipticalTorusXY : public DynaMesh { public:
    // Generate an elliptical torus in the XY plane, Can be rotated
    EllipticalTorusXY( float a, float b, float dia, uint rotationRes, uint revolveRes, Color color = BLUE ) : DynaMesh( rotationRes * revolveRes * 2 ) {

        Vector3 circCntr_i, circCntr_ip1, axis_i, axis_ip1;
        Vector3 rad_i, rad_ip1;
        Vector3 p1, p2, p3, p4, n1, n2, n3, n4;

        float theta   = 0.0f, phi;
        float rotStep = (2.0f * M_PI) / (1.0f * rotationRes);
        float revStep = (2.0f * M_PI) / (1.0f * revolveRes );

        for( uint i = 0; i < rotationRes; ++i ){

            circCntr_i   = {a*cosf(theta)        , b*sinf(theta)        , 0.0f};
            circCntr_ip1 = {a*cosf(theta+rotStep), b*sinf(theta+rotStep), 0.0f};
            axis_i /*-*/ = Vector3CrossProduct( Vector3{0.0f,0.0f,-1.0f}, circCntr_i   );
            axis_ip1     = Vector3CrossProduct( Vector3{0.0f,0.0f,-1.0f}, circCntr_ip1 );
            phi /*----*/ = 0.0f;
            rad_i /*--*/ = Vector3Scale( Vector3Normalize( circCntr_i   ), dia/2.0f );
            rad_ip1 /**/ = Vector3Scale( Vector3Normalize( circCntr_ip1 ), dia/2.0f );

            for( uint j = 0; j < revolveRes; ++j ){
                p1 = Vector3Add( circCntr_i  , Vector3RotateByAxisAngle( rad_i  , axis_i  , phi         ) );
                p2 = Vector3Add( circCntr_ip1, Vector3RotateByAxisAngle( rad_ip1, axis_ip1, phi         ) );
                p3 = Vector3Add( circCntr_i  , Vector3RotateByAxisAngle( rad_i  , axis_i  , phi+revStep ) );
                p4 = Vector3Add( circCntr_ip1, Vector3RotateByAxisAngle( rad_ip1, axis_ip1, phi+revStep ) );
                n1 = Vector3Normalize( Vector3Subtract( p1, circCntr_i   ) );
                n2 = Vector3Normalize( Vector3Subtract( p2, circCntr_ip1 ) );
                n3 = Vector3Normalize( Vector3Subtract( p3, circCntr_i   ) );
                n4 = Vector3Normalize( Vector3Subtract( p4, circCntr_ip1 ) );

                tris.push_back( {p3, p1, p2} );
                nrms.push_back( {n3, n1, n2} );
                
                tris.push_back( {p3, p2, p4} );
                nrms.push_back( {n3, n2, n4} );

                phi += revStep;
            }
            theta += rotStep;
        }
        // 3. Set color
        set_uniform_color( color );
        load_mesh_buffers( true, true );
    }
};

struct PlanetOrbitMap{
    float theta;
    float angStep;
    uint  planDex;
    float a;
    float b;
};

class StarSystemMap : public CompositeModel {
    // Fanciful 3D representation of a star system

    /// Members ///
    Vector3 orbitNormal;
    ubyte   N_planets;
};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Ancient Star Map" );
    SetTargetFPS( 60 );
    // rlDisableBackfaceCulling();

    ///// Create Objects //////////////////////////////////////////////////

    /// Create Camera ///
    Camera camera = Camera{
        Vector3{   0.5,   0.5,  25.0 }, // Position
        Vector3{   0.0,   0.0,   0.0 }, // Target
        Vector3{   0.0,   0.0,   1.0 }, // Up
        45.0, // ---------------------- FOV_y
        0 // -------------------------- Projection mode
    };

    /// Lighting ///
    Lighting lightShader{};
    lightShader.set_camera_posn( camera );

    /// Components ///
    Icosahedron icos{ 5.0f, Vector3Zero(), GREEN };
    icos.set_shader( lightShader.shader );

    Sphere sphr{ 5.0f, Vector3Zero(), 8, GREEN };
    sphr.set_shader( lightShader.shader );

    EllipticalTorusXY ellipse{ 6, 8, 1.00, 50, 6, GREEN };
    ellipse.set_shader( lightShader.shader );

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode3D( camera );
        ClearBackground( BLACK );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        lightShader.update();

        // icos.update();
        sphr.draw();
        ellipse.draw();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}

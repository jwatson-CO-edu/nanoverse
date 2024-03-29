// g++ 30_star-map.cpp -std=c++17 -lraylib -O3 -o starMap.out
// Animated star-map with normal maps, Inspired by the Ahsoka end-credits sequence


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

float vsnf( float angle ){  return 1.0f - cosf( angle );  } // Versine, float version
    

Matrix MatrixRotateAxisAngle( const Vector3& axis, float angle_rad ){
    // Return a homogeneous transform that is a rotation by `angle_rad` about the `axis`
    Matrix rtnMtx = MatrixIdentity();
    /* m0 m4 m8  m12
       m1 m5 m9  m13
       m2 m6 m10 m14
       m3 m7 m11 m15 */
    // 1. Calc components
    Vector3 axis_ = Vector3Normalize( axis );
    float   k1    = axis_.x;
    float   k2    = axis_.y;
    float   k3    = axis_.z;
    float   vTh   = vsnf( angle_rad );
    float   cTh   = cosf( angle_rad );
    float   sTh   = sinf( angle_rad );
    // 2. X-basis
    rtnMtx.m0 = k1*k1*vTh + cTh;
    rtnMtx.m1 = k2*k1*vTh + k3*sTh;
    rtnMtx.m2 = k3*k1*vTh - k2*sTh;
    // 3. Y-basis
    rtnMtx.m4 = k1*k2*vTh - k3*sTh;
    rtnMtx.m5 = k2*k2*vTh + cTh;
    rtnMtx.m6 = k3*k2*vTh + k1*sTh;
    // 4. Z-basis
    rtnMtx.m8  = k1*k3*vTh + k2*sTh;
    rtnMtx.m9  = k2*k3*vTh - k1*sTh;
    rtnMtx.m10 = k3*k3*vTh + cTh;
    // N. Return
    return rtnMtx;
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
        // NOTE: This is a building block for the subdivided sphere

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
        // Compute the vertices and faces
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
        // Compute the vertices and faces
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



////////// STAR SYSTEM MAP /////////////////////////////////////////////////////////////////////////

struct PlanetOrbitMap{
    // State for one planet and its orbit
    // NOTE: Gyroscopic procession not currently modeled!

    /// System Info ///
    uint planDex; // Inex of the planet in the star system

    /// Orbit Info ///
    float   theta; // ---- Progress of planet along its orbit
    float   rotStep; // -- Angular step of orbit
    Vector3 orbitNorm; //- Normal of orbital plane
    float   ZrotOrbit; //- Rotation of orbit about its normal
    float   a; // -------- Axis 1 of orbit ellipse
    float   b; // -------- Axis 2 of orbit ellipse
    float   f; // -------- Distance from center to each focus
    Vector3 orbitFocus; // Displacement from ellipse center to orbit center

    /// Revolution Info ///
    float   phi; // ----- Revolution angle of planet
    float   revStep; // - Angular step of revolution
    Vector3 revolNorm; // Revolution axis of the planet

    Vector3 get_current_posn(){
        // Get the planet's position from the current `theta`
        // return Vector3Subtract( Vector3{a*cosf(theta), b*sinf(theta), 0.0f}, orbitFocus );
        return Vector3{a*cosf(theta), b*sinf(theta), 0.0f};
    }

    Matrix get_current_pose(){
        // Get the planet's pose from the current `theta` and `phi`
        return set_posn(
            MatrixMultiply(
                MatrixRotateAxisAngle(
                    Vector3Normalize( Vector3CrossProduct( Vector3{0.0f, 0.0f, 1.0f}, revolNorm ) ),
                    Vector3Angle( Vector3{0.0f, 0.0f, 1.0f}, revolNorm )
                ),
                MatrixRotateZ( phi )
            ),
            get_current_posn()
        );
    }

    void update(){
        // Update the rotation and revolution of the planet
        theta += rotStep;
        phi   += revStep;
    }
};

struct SunGlyph{
    // 2D Billboard of a star, because you typically cannot visit them

    /// Members ///
    // 2D State
    int   discDia;
    int   miniRad;
    float scale;
    float theta;
    float stepTh;
    // 3D State
    Vector3 posn;
    Vector3 up;
    // Texture
    Rectangle /*-*/ source;
    RenderTexture2D glyph;
    
    /// Drawing ///

    void update_texture(){
        // Step the animated texture representing the star
        theta += stepTh;
        float theta_i = theta;

        BeginTextureMode( glyph );

            // Wipe image && Draw Disc
            ClearBackground( {0,0,0,0} );
            DrawCircle( (int)(1.5*discDia), (int)(1.5*discDia), (int)(1.2f*discDia/2), BLACK  );
            DrawCircle( (int)(1.5*discDia), (int)(1.5*discDia), discDia/2            , YELLOW );

            // Draw rays
            int rad_j = miniRad;
            int x, y, hyp;

            for( ubyte i = 0; i < 6; ++i ){
                rad_j = miniRad;
                hyp    = (int)(discDia/2 + rad_j*3);
                for( ubyte j = 0; j < 3; ++j ){
                    x = (int)(1.5*discDia) + (int)( hyp * cosf( theta_i ) );
                    y = (int)(1.5*discDia) + (int)( hyp * sinf( theta_i ) );
                    DrawCircle( x, y, (int)(1.2f*rad_j), BLACK  );
                    DrawCircle( x, y, rad_j            , YELLOW );
                    rad_j = (int)( rad_j*0.75 );
                    hyp   += (int)( rad_j*3 );
                }
                theta_i += M_PI / 3.0;
            }
        EndTextureMode();
    }

    /// Constructor(s) ///

    SunGlyph(){
        discDia = 50;
        theta   = 0.0f;
        stepTh  = M_PI/420.0f;
        miniRad = (int)(discDia/8.0f);
        glyph   = LoadRenderTexture( 3*discDia, 3*discDia );
        BeginTextureMode( glyph );
            DrawCircle( (int)(1.5*discDia), (int)(1.5*discDia), (int)(1.2f*discDia/2), BLACK  );
            DrawCircle( (int)(1.5*discDia), (int)(1.5*discDia), discDia/2            , YELLOW );
        EndTextureMode();
        source = { 0.0f, 0.0f, (float)glyph.texture.width, (float)glyph.texture.height };
        posn   = Vector3Zero();
        up     = Vector3{0,0,1};
    }

    SunGlyph( int discDia_, const Vector3& location ){
        discDia = discDia_; 
        theta   = 0.0f;
        stepTh  = M_PI/420.0f;
        miniRad = (int)(discDia/8.0f);
        glyph   = LoadRenderTexture( 3*discDia, 3*discDia );
        BeginTextureMode( glyph );
            DrawCircle( (int)(1.5*discDia), (int)(1.5*discDia), (int)(1.2f*discDia/2), BLACK  );
            DrawCircle( (int)(1.5*discDia), (int)(1.5*discDia), discDia/2            , YELLOW );
        EndTextureMode();
        source = { 0.0f, 0.0f, (float)glyph.texture.width, (float)glyph.texture.height };
        posn   = location;
    }

    /// Methods ///

    void draw( Camera camera ){
        // Draw one frame of the animated star texture
        DrawBillboardPro( camera, glyph.texture, source, posn, camera.up, 
                          (Vector2) {3.0f, 3.0f}, (Vector2) {0.0f, 0.0f}, 0.0f, WHITE );
    }
};

class StarSystemMap : public CompositeModel { public:
    // Fanciful 3D representation of a star system

    /// Members ///
    ubyte /*------------*/ N_planets; // Number of planets in this star system
    vector<PlanetOrbitMap> orbits; // -- State for each planet's orbit
    float /*------------*/ pathDia;
    SunGlyph /*---------*/ star;

    /// Constructor(s) ///

    StarSystemMap(){
        // Default constructor
        N_planets = 0;
        pathDia   = 0.15;
        star /**/ = SunGlyph{ 500, Vector3Zero() };
    }

    StarSystemMap( const Matrix& pose ){
        // Default constructor
        set_pose( pose );
        N_planets = 0;
        pathDia   = 0.15;
        star /**/ = SunGlyph{ 500, Vector3Zero() };
    }

    /// Methods ///

    void add_planet( float planetRad, const Vector3& planetAxis, float initPhi,
                     float orbitAxis1Len, float orbitAxis2Len, const Vector3& orbitNorm, float orbitZrot, float initTheta, 
                     float thetaStep ){
        // 0. Increment planets
        N_planets++;

        // 1. Create state
        PlanetOrbitMap orbit{};
        orbit.theta     = initTheta;
        orbit.a /*---*/ = orbitAxis1Len;
        orbit.b /*---*/ = orbitAxis2Len;
        orbit.f /*---*/ = sqrtf( abs( orbit.a*orbit.a - orbit.b*orbit.b ) );
        orbit.orbitNorm = Vector3Normalize( orbitNorm  );
        orbit.revolNorm = Vector3Normalize( planetAxis );
        orbit.ZrotOrbit = orbitZrot;
        orbit.rotStep   = thetaStep;
        orbit.revStep   = M_PI / 180.f;
        
        // 2. Set the focus
        if( orbit.a > orbit.b )  orbit.orbitFocus = {orbit.f, 0.0f, 0.0f};  else  orbit.orbitFocus = {0.0f, orbit.f, 0.0f};
        
        // 3. Set the frame
        Matrix orbitDisp  = set_posn( MatrixIdentity(), orbit.orbitFocus );
        Matrix orbitXform = MatrixMultiply(
            orbitDisp,
            MatrixMultiply(
                MatrixRotateAxisAngle(
                    Vector3Normalize( Vector3CrossProduct( Vector3{0.0f, 0.0f, 1.0f}, orbit.orbitNorm ) ),
                    Vector3Angle( Vector3{0.0f, 0.0f, 1.0f}, orbit.orbitNorm )
                ),
                MatrixRotateZ( orbit.ZrotOrbit )
            )
        );
        
        // 4. Create planet
        Vector3 initPosn = Vector3Zero();
        dynaPtr planet   = dynaPtr( new Sphere{ planetRad, initPosn, 5, GOLD } );
        planet->Trel = orbitXform;
        planet->Tcur = orbit.get_current_pose();
        orbit.planDex = parts.size();
        add_component( planet );

        // 5. Create orbit
        dynaPtr path = dynaPtr( new EllipticalTorusXY{ orbitAxis1Len, orbitAxis2Len, pathDia, 50, 6, GOLD } );
        path->Trel = orbitXform;
        path->Tcur = MatrixIdentity();
        add_component( path );

        // 5. Save orbit state
        orbits.push_back( orbit );
    }

    void generate_system( ubyte maxPlanets = 12 ){
        ubyte   Ngen = min( maxPlanets, (ubyte)(randi(1, 6) + randi(1, 6)) ); // Roll 2d6 = Number of planets to generate
        float   radMax = 0.0f;
        float   a, b, planRad, orbZrot, bgnTh, stepTh;
        Vector3 orbNorm;
        for( ubyte i = 0; i < Ngen; ++i ){
            // 1. Generate orbit
            orbZrot = randf( 0.0f, 2.0f*M_PI );
            bgnTh   = randf( 0.0f, 2.0f*M_PI );
            a /*-*/ = radMax + randf( 2.0f, 4.0f );
            b /*-*/ = radMax + randf( 2.0f, 4.0f );
            radMax  = max( a, b );
            // Small chance of highly eccentric
            if( rand_ubyte() <= 2 ){  b += randf( 2.0f, 4.0f );  }
            orbNorm = Vector3Normalize( uniform_vector_noise( {0,0,1}, 0.1f ) );
            // Small chance of highly obtuse orbital plane
            if( rand_ubyte() <= 2 ){  orbNorm = Vector3Normalize( uniform_vector_noise( orbNorm, 2.0f ) );  }
            stepTh = M_PI/( 60.f + 120.0f*i + randf( 0.0f, 120.0f ) );
            // Small chance of retrograde orbit
            if( rand_ubyte() <= 2 ){  stepTh *= -1.0f;  }
            // 2. Generate planet
            planRad = randf( 0.3f, 0.3f+1.5f*randf()*(i+1.0f)/(Ngen*1.0f)  );
            // 3. Instantiate Planet
            add_planet( planRad, Vector3{0,0,1}, 0.0f,
                        a, b, orbNorm, orbZrot, bgnTh, 
                        stepTh );
        }
    }

    void update(){
        // Advance each planet in its orbit
        for( PlanetOrbitMap& orbit : orbits ){
            orbit.update();
            parts[ orbit.planDex ]->Tcur = orbit.get_current_pose();
        }
        set_part_poses();
    }

    void draw_glyphs( Camera camera ){
        // Draw flat UI elements, including the star
        star.draw( camera );
    }

    float get_greatest_radius(){
        // Get the furthest extent of any orbit in any direction
        float radMax = -1e6;
        float rad_i;
        for( PlanetOrbitMap& orbit : orbits ){
            rad_i = sqrtf( orbit.a*orbit.a + orbit.b*orbit.b );
            if( rad_i > radMax )  radMax = rad_i;
        }
        return radMax;
    }

};

////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 1200, 900, "Ancient Star Map" );
    SetTargetFPS( 60 );
    // rlDisableBackfaceCulling();

    ///// Create Objects //////////////////////////////////////////////////

    /// Create Camera ///
    Vector3 camAxis = Vector3Normalize( {0.25, 0.0, 1.0} );
    Vector3 camStik{ 25.0f, 0.0f, 0.0f };
    float   camAngl = 0.0f;
    float   camStep = M_PI / 720.0f;
    Camera  camera  = Camera{
        camStik, // -------------- Position
        Vector3{0.0, 0.0, 0.0}, // Target
        camAxis, // -------------- Up
        45.0, // ----------------- FOV_y
        0 // --------------------- Projection mode
    };

    /// Lighting ///
    Lighting lightShader{};
    lightShader.light.position = Vector3Zero(); // FIXME: WRITE LITE POSN SETTER
    lightShader.set_camera_posn( camera );

    /// Components ///
    StarSystemMap map{};
    map.generate_system();
    // map.add_planet( 0.40, Vector3{0.0,0.0,1.0}, 0.0, 3.0,  3.0, Vector3{0.0 ,0.0 ,1.0}, 0.0, 0.0     , M_PI/180.f );
    // map.add_planet( 0.80, Vector3{0.0,0.0,1.0}, 0.0, 7.0,  8.0, Vector3{0.0 ,0.25,1.0}, 1.5, M_PI/2.0, M_PI/300.f );
    // map.add_planet( 0.70, Vector3{0.0,0.0,1.0}, 0.0, 9.0, 13.0, Vector3{0.15,0.00,1.0}, 3.0, M_PI    , M_PI/420.f );
    map.set_shader( lightShader.shader );

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        map.star.update_texture();

        /// Begin Drawing ///
        BeginDrawing();
        ClearBackground( BLACK );
        BeginMode3D( camera );
        

        ///// DRAW LOOP ///////////////////////////////////////////////////

        camAngl += camStep;
        camera.position = Vector3RotateByAxisAngle( camStik, camAxis, camAngl );
        lightShader.set_camera_posn( camera );

        lightShader.update();

        map.update();
        map.draw_glyphs( camera );
        map.draw();

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}

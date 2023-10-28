// g++ 31_star-map.cpp -std=c++17 -lraylib -O3 -o starMap.out
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
    uint  planDex; // Inex of the planet in the star system
    ulong ts = 0;

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
        return Vector3{a*cosf(theta), b*sinf(theta), 0.0f};
    }

    Vector3 get_posn( float theta_ ){
        // Get the planet's position from the given `theta_`
        return Vector3{a*cosf(theta_), b*sinf(theta_), 0.0f};
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

    Matrix get_pose( float theta_ ){
        // Get the planet's pose from the current `theta_` and `phi`
        return set_posn(
            MatrixMultiply(
                MatrixRotateAxisAngle(
                    Vector3Normalize( Vector3CrossProduct( Vector3{0.0f, 0.0f, 1.0f}, revolNorm ) ),
                    Vector3Angle( Vector3{0.0f, 0.0f, 1.0f}, revolNorm )
                ),
                MatrixRotateZ( phi )
            ),
            get_posn( theta_ )
        );
    }

    void update(){
        // Update the rotation and revolution of the planet
        theta += rotStep;
        phi   += revStep;
        ++ts;
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

    void draw( const Camera& camera ){
        // Draw one frame of the animated star texture
        Matrix matView = rlGetMatrixModelview();
        DrawBillboardPro( camera, glyph.texture, source, posn, Vector3{ matView.m1, matView.m5, matView.m9 }, 
                          (Vector2) {3.0f, 3.0f}, (Vector2) {0.0f, 0.0f}, 0.0f, WHITE );
    }
};

struct OrbitSchedule{
    // A point along an orbit and when the planet will get there
    Vector3 absPnt; // A point of interest along an orbit
    ulong   nextTs; // The number of timesteps until arrival at the point
    ulong   period; // The length of one orbit in timesteps
};

class StarSystemMap : public CompositeModel { public:
    // Fanciful 3D representation of a star system

    /// Members ///
    ubyte /*------------*/ N_planets; // Number of planets in this star system
    vector<PlanetOrbitMap> orbits; // -- State for each planet's orbit
    float /*------------*/ pathDia; // - Diameter of the elliptical torus visually representing the orbit
    SunGlyph /*---------*/ star; // ---- Animated texture of a star UI element
    ulong /*------------*/ ts; // ------ Current timestep of this system (Should be same as global ...)

    /// Constructor(s) ///

    StarSystemMap(){
        // Default constructor
        N_planets = 0;
        pathDia   = 0.15;
        star /**/ = SunGlyph{ 500, Vector3Zero() };
        ts /*--*/ = 0;
    }

    StarSystemMap( const Matrix& pose ){
        // Default constructor
        set_pose( pose );
        N_planets = 0;
        pathDia   = 0.15;
        star /**/ = SunGlyph{ 500, get_posn( pose ) };
        ts /*--*/ = 0;
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
        ++ts;
    }

    void draw_glyphs( const Camera& camera ){
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

    Vector3 get_i_position( ubyte i, float theta_ ){
        // Get the absolute position of planet `i` at angular position `theta_` 
        float   oldTheta;
        Vector3 rtnVec;
        if( i < N_planets ){
            oldTheta = orbits[i].theta;
            parts[ orbits[i].planDex ]->Tcur = orbits[i].get_pose( theta_ );
            rtnVec = get_posn( set_part_pose( orbits[i].planDex ) );
            parts[ orbits[i].planDex ]->Tcur = orbits[i].get_pose( oldTheta );
            return rtnVec;
        }
        return Vector3Error();
    }

    Vector3 get_i_position( ubyte i ){
        // Get the absolute position of planet `i` at angular position `theta_` 
        if( i < N_planets ){
            parts[ orbits[i].planDex ]->Tcur = orbits[i].get_current_pose();
            return get_posn( set_part_pose( orbits[i].planDex ) );
        }
        return Vector3Error();
    }

    OrbitSchedule get_closest_point( ubyte i, const Vector3& query ){
        // Get the point on the orbit closest to the `query` in the world frame 
        OrbitSchedule rtnSched{};
        rtnSched.absPnt = Vector3Error();
        rtnSched.nextTs = 0;
        rtnSched.period = 0;

        float   oldTheta;
        float   stepTh;
        float   distSqr;
        float   dSqrMin = 1e9;
        float   delta   =   0.0f;
        ulong   tDiff   = 0;
        Vector3 posn_i;
        
        if( i < N_planets ){
            oldTheta /*--*/ = orbits[i].theta;
            stepTh /*----*/ = orbits[i].rotStep;
            rtnSched.period = (ulong) ceilf( 2.0f*M_PI/stepTh );
            while( delta <= (2.0f*M_PI) ){
                posn_i  = get_i_position( i, oldTheta+delta );
                distSqr = Vector3DistanceSqr( posn_i, query );
                if( distSqr < dSqrMin ){
                    dSqrMin /*---*/ = distSqr;
                    rtnSched.absPnt = posn_i;
                    rtnSched.nextTs = orbits[i].ts + tDiff;
                }
                delta += stepTh;
                ++tDiff;
            }
        }

        return rtnSched;
    }
};
typedef shared_ptr<StarSystemMap> ptrSysMap;


////////// PATH / ROUTING //////////////////////////////////////////////////////////////////////////

struct Path{
    // One edge of a map, Represents a trip between two planets

    /// Members ///

    Vector3 bgnVec; // Beginning of 3D path
    Vector3 endVec; // End of 3D path
    Vector3 cursor; // Current point along the path
    Vector3 dirVec; // Normalized direction of the path
    ulong   bgnTs; //- Global timestep of journey beginning
    ulong   endTs; //- Global timestep of journey end
    float   dStep; //- Linear distance traveled per timestep
    Vector3 vStep; //- Linear offset   accrued  per timestep

    /// Constructor(s) ///

    Path( const Vector3& beginVec, const Vector3& endVec_, ulong beginTs = 0 ){
        bgnVec = beginVec; // Beginning of 3D path
        endVec = endVec_; // End of 3D path
        dirVec = Vector3Normalize( Vector3Subtract( endVec, bgnVec ) );
        cursor = beginVec; // Current point along the path
        bgnTs  = beginTs; //- Global timestep of journey beginning
    }

    /// Methods ///

    void set_speed( float distPerTs ){
        // Set params for a given linear per-frame speed
        float dTotl = Vector3Distance( bgnVec, endVec );
        /*-*/ dStep = distPerTs;
        /*-*/ vStep = Vector3Scale( dirVec, dStep );
        ulong Nstep = (ulong) ceilf( dTotl / dStep );
        /*-*/ endTs = bgnTs + Nstep;
    }

    Vector3 update(){
        // Set the cursor for the next frame && Return the current position along the path
        float dRemSqr = Vector3DistanceSqr( cursor, endVec );
        // If there is path remaining, then update the cursor
        if( dRemSqr > 0.0f ){
            if( dRemSqr > (dStep*dStep) )
                cursor = Vector3Add( cursor, vStep );
            else  
                cursor = endVec;
        } // Else no update
        // Return the current position along the path
        return cursor;
    }

    void draw(){
        // Render the path
        // 2023-10-27: Draw in the simplest way until you are ready to render for realsies
        DrawLine3D( bgnVec, cursor, GOLD );
    }
};  

Path chart_course_between_planets( ptrSysMap sys1, ulong i, ptrSysMap sys2, ulong j, float targetSpeed ){
    // Get a path between the present position of the start and the closest position of the destination
    Vector3 /*-*/ bgnPoint  = sys1->get_i_position(i);
    OrbitSchedule sched     = sys2->get_closest_point( j, bgnPoint );
    float /*---*/ dist /**/ = Vector3Distance( bgnPoint, sched.absPnt );
    float /*---*/ leastDiff = 1e6;
    ulong /*---*/ k /*---*/ = 0;
    float /*---*/ speed_k   = 1e3;
    float /*---*/ diff_k    = 1e3;
    Path /*----*/ rtnPath{ bgnPoint, sched.absPnt };
    
    while( diff_k < leastDiff ){

        leastDiff = diff_k;
        rtnPath.set_speed( speed_k );
        
        speed_k   = dist / (1.0f * (sched.nextTs - sys2->ts + k*sched.period));
        diff_k    = abs( speed_k - targetSpeed );
        ++k;
    }
    // FIXME, START HERE: CHOOSE THE POINT IN THE FUTURE CLOSEST TO THE TARGET SPEED
}



////////// CAMERA //////////////////////////////////////////////////////////////////////////////////

class DragOffsetThirdP_Camera : public Camera3D{ public:
	// Aircraft drags the camera like in games

	Vector3 trgtCenter; //- Position of the target
	Vector3 dragCenter; //- "Weight" being "drug" by the target
	float   offset_d; // -- Desired camera offset in meters
    Vector3 absDragOfst; // Offset vector from the drag point
    Vector3 absTrgtOfst; // Offset vector from the look point

    static constexpr float dDrawMin = RL_CULL_DISTANCE_NEAR;
    static constexpr float dDrawMax = RL_CULL_DISTANCE_FAR;

    DragOffsetThirdP_Camera( float desiredOffset_m, const Vector3& dragOffsetAbs, const Vector3& targetOffsetAbs ) : Camera3D(){
        // Set follower params 
        trgtCenter  = Vector3Zero(); 
        dragCenter  = Vector3Zero(); 
		offset_d    = desiredOffset_m;
        absDragOfst = dragOffsetAbs;
        absTrgtOfst = targetOffsetAbs;
        // Set inherited params
        position   = Vector3Add( dragCenter, absDragOfst );
        target     = Vector3Add( trgtCenter, absTrgtOfst );
        up /*---*/ = Vector3{ 0.0, 0.0, 1.0 };
        fovy /*-*/ = 45.0f;
        projection = 0;
    }

    void advance_camera(){
		// Move the camera after all the target updates are in
        Vector3 trgtDiff = Vector3Subtract( dragCenter, trgtCenter );
        float   sepDist  = min( Vector3Length( trgtDiff ), offset_d );
		Vector3 dragVec  = Vector3Scale(  Vector3Normalize( trgtDiff ), sepDist  );
		dragCenter = Vector3Add( trgtCenter, dragVec );
        // Set the offset positions
		target   = Vector3Add( trgtCenter, absTrgtOfst );
        position = Vector3Add( dragCenter, absDragOfst );
	}

    void update_target_position( Vector3 tCenter ){  
        // Point the camera (without offset)
        trgtCenter = tCenter;  
        advance_camera();
    } 

    bool inside_FOV( const Vector3& pnt ) const{
        // Return true if the ray from the camera to the `pnt` is within the FOV (conservative)
        Vector3 vLook = Vector3Subtract( target, position );
        Vector3 vRayP = Vector3Subtract( pnt   , position );
        if( Vector3Angle( vLook, vRayP ) * RAD2DEG <= fovy )  
            return true;
        else
            return false;
    }

    float signed_distance_to_drawn_sphere( const Vector3& pnt ) const{
        // Signed distance to a sphere that has the max draw distance as a radius
        if( inside_FOV( pnt ) ){
            return Vector3Distance( pnt, position ) - dDrawMax;
        }else{
            return nanf("");
        }
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
    
    Vector3 posn1{0,0,0};
    Vector3 posn2{100.0f, 0.0f, 0.0f};

    /// Create Camera ///
    DragOffsetThirdP_Camera camera = DragOffsetThirdP_Camera{ 10.0f, Vector3{0,0,5}, Vector3{0,0,2} };
    camera.dragCenter = Vector3{50,50,50};
    Path path{ posn1, posn2 };
    path.set_speed( 0.06125f );
    camera.update_target_position( path.cursor );

    /// Lighting ///
    Lighting lightShader{};
    lightShader.light.position = Vector3Zero(); // FIXME: WRITE LITE POSN SETTER
    lightShader.set_camera_posn( camera );

    /// Components ///
    StarSystemMap sys1{ set_posn( MatrixIdentity(), posn1 ) };
    sys1.generate_system();
    sys1.set_shader( lightShader.shader );

    StarSystemMap sys2{ set_posn( MatrixIdentity(), posn2 ) };
    sys2.generate_system();
    sys2.set_shader( lightShader.shader );

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Updates *outside* the 3D context ///
        sys1.star.update_texture();
        sys2.star.update_texture();
        path.update();
        camera.update_target_position( path.cursor );

        /// Begin Drawing ///
        BeginDrawing();
        ClearBackground( BLACK );
        BeginMode3D( camera );
        

        ///// DRAW LOOP ///////////////////////////////////////////////////

        lightShader.set_camera_posn( camera );

        lightShader.update();

        path.draw();

        sys1.update();
        sys1.draw_glyphs( camera );
        sys1.draw();

        sys2.update();
        sys2.draw_glyphs( camera );
        sys2.draw();


        ///// END LOOP ////////////////////////////////////////////////////

        /// End Drawing ///
        EndMode3D();
        EndDrawing();
    }

    return 0;
}

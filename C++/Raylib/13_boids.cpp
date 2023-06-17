// g++ 13_boids.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <algorithm>
using std::clamp;

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"



////////// HELPER STRUCTS //////////////////////////////////////////////////////////////////////////

struct Basis{
    // Pose exchange format for `Boid`s, Z-basis has primacy
    // NOTE: None of the operations with other bases assume any operand is orthonormalized
    // NOTE: Position is largely absent from Basis-Basis operations

    /// Members ///
    Vector3 Xb; // X-basis
    Vector3 Yb; // Y-basis
    Vector3 Zb; // Z-basis
    Vector3 Pt; // Position

    /// Static Methods ///

    static Basis origin_Basis(){
        // Get the origin `Basis`
        Basis rtnBasis;
        rtnBasis.Xb = Vector3{ 1.0f, 0.0f, 0.0f };
        rtnBasis.Yb = Vector3{ 0.0f, 1.0f, 0.0f };
        rtnBasis.Zb = Vector3{ 0.0f, 0.0f, 1.0f };
        rtnBasis.Pt = Vector3{ 0.0f, 0.0f, 0.0f };
        return rtnBasis;
    }

    static Basis random_Basis(){
        // Sample from a +/-1.0f cube for all vectors, then `orthonormalize`
        Basis rtnBasis;
        rtnBasis.Xb = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.Yb = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.Zb = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.Pt = Vector3{ randf( -1.0,  1.0 ), randf( -1.0,  1.0 ), randf( -1.0,  1.0 ) };
        rtnBasis.orthonormalize();
        return rtnBasis;
    }

    /// Methods ///

    void orthonormalize(){
        // Make sure this is an orthonormal basis, Z-basis has primacy
        // No need to normalize `Xb`, see below
        Yb = Vector3Normalize( Yb );
        Zb = Vector3Normalize( Zb );
        Xb = Vector3Normalize( Vector3CrossProduct( Yb, Zb ) );
        Yb = Vector3Normalize( Vector3CrossProduct( Zb, Xb ) );
    };

    void blend_orientations_with_factor( const Basis& other, float factor ){
        // Exponential filter between this basis and another Orthonormalize separately
        // 1. Clamp factor
        factor = clamp( factor, 0.0f, 1.0f );
        // 2. Blend bases
        // No need to compute `Xb`, see `orthonormalize`
        Yb = Vector3Add(  Vector3Scale( Yb, 1.0-factor ), Vector3Scale( other.Yb, factor )  );
        Zb = Vector3Add(  Vector3Scale( Zb, 1.0-factor ), Vector3Scale( other.Zb, factor )  );
        // Pt = Vector3Add(  Vector3Scale( Pt, 1.0-factor ), Vector3Scale( other.Pt, factor )  );
        // 3. Correct basis
        orthonormalize();
    }

    Basis operator+( const Basis& other ){
        // Addition operator for bases, Just the vector sum of each basis, Orthonormalize separately
        Basis rtnBasis;
        rtnBasis.Xb = Vector3Add( Xb, other.Xb );
        rtnBasis.Yb = Vector3Add( Yb, other.Yb );
        rtnBasis.Zb = Vector3Add( Zb, other.Zb );
        rtnBasis.Pt = Vector3Add( Pt, other.Pt );
        return rtnBasis;
    }

    Basis get_scaled_orientation( float factor ){
        // Return a copy of the `Basis` with each individual basis scaled by a factor
        Basis rtnBasis;
        rtnBasis.Xb = Vector3Scale( Xb, factor );
        rtnBasis.Yb = Vector3Scale( Yb, factor );
        rtnBasis.Zb = Vector3Scale( Zb, factor );
        // rtnBasis.Pt = Vector3Scale( Pt, factor );
        return rtnBasis;
    }
};
typedef shared_ptr<Basis> basPtr; // 2023-06-17: For now assume that Bases are lightweight enough to pass by value

Basis basis_from_transform_and_point( const Matrix& xform, const Vector3& point ){
    // Get a `Basis` from a `xform` matrix and a `point`
    Basis rtnBasis;
    rtnBasis.Xb = Vector3Transform( Vector3{1.0, 0.0, 0.0}, xform );
    rtnBasis.Yb = Vector3Transform( Vector3{0.0, 1.0, 0.0}, xform );
    rtnBasis.Zb = Vector3Transform( Vector3{0.0, 0.0, 1.0}, xform );
    rtnBasis.Pt = point;
    return rtnBasis;
}


////////// TOYS ////////////////////////////////////////////////////////////////////////////////////


class Boid : public TriModel{ public:
    // A flocking entity like a bird

    /// Members ///
    float   length; // - Length of the boid
    float   width; // -- Width of the boid
    Color   sldClr; // - Boid color
    float   dNear; // -- Radius of hemisphere for flocking consideration
    Vector3 home; // --- Don't get too far from this point

    /// Way-Finding ///
    Basis flocking; // Flocking instinct
    Basis homeSeek; // Home seeking instinct
    Basis freeWill; // Drunken walk
    Basis headingB; // Where the boid is actually pointed

    /// Constructor ///

    Boid( float len, float wdt ) : TriModel( 2 ){
        // Build the geometry of the boid
        length = len;
        width  = wdt;
        dNear  = length * 5.0f;
        sldClr = Color{
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            (ubyte) randi( 0, 255 ),
            255
        };
        flocking = Basis::origin_Basis(); // Flocking instinct
        homeSeek = Basis::origin_Basis(); // Home seeking instinct
        freeWill = Basis::origin_Basis(); // Drunken walk
        headingB = Basis::random_Basis(); // Where the boid is actually pointed

        load_tri( // Horizontal Plane
            Vector3{  width/2, 0.0f, 0.0f   }, 
            Vector3{ -width/2, 0.0f, 0.0f   }, 
            Vector3{  0.0f   , 0.0f, length }
        );
        load_tri( // Vertical Plane
            Vector3{  0.0f,  width/2, 0.0f   }, 
            Vector3{  0.0f, -width/2, 0.0f   }, 
            Vector3{  0.0f, 0.0f    , length }
        );
    }

    Basis get_Basis(){
        // Get pose info for this boid
        return basis_from_transform_and_point( T, XYZ );
    }

    /// Navigation ///

    Basis consider_neighbors( const vector<Basis>& nghbrPoses ){
        // Flocking instinct: Adjust bearing according to the closest neighbors in front of the boid
        Vector3 Xmean;
        Vector3 Ymean;
        Vector3 Zmean;
        Vector3 Zfly , diffVec;
        Basis   rtnMsg;
        uint    relevant;
        float   dist, dotFront;
        Xmean = Vector3{0.0f, 0.0f, 0.0f};
        Ymean = Vector3{0.0f, 0.0f, 0.0f};
        Zmean = Vector3{0.0f, 0.0f, 0.0f};
        for( Basis msg : nghbrPoses ){
            diffVec  = Vector3Subtract( msg.Pt, XYZ );
            dist     = Vector3Length( diffVec );
            if( dist > 0.0 ){
                Zfly     = Vector3Transform( Vector3{0.0f, 0.0f, 1.0f}, T );
                dotFront = Vector3DotProduct( Zfly, diffVec );
                if( (dist <= dNear) || (dotFront >= 0.0f) ){
                    Xmean  = Vector3Add( Xmean, msg.Xb );
                    Ymean  = Vector3Add( Ymean, msg.Yb );
                    Zmean  = Vector3Add( Zmean, msg.Zb );
                    relevant++;
                }
            }
        }
        if( relevant ){
            rtnMsg.Xb = Xmean;
            rtnMsg.Yb = Ymean;
            rtnMsg.Zb = Zmean;
            rtnMsg.orthonormalize();
        }else{
            rtnMsg.Xb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Yb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Zb = Vector3{0.0f, 0.0f, 0.0f};
        }
        return rtnMsg;
    }

    Basis consider_home( uint N_samples ){
        // Home seeking instinct: Take `N_samples` and choose the one that points closest to `home`
        Basis   rtnMsg;
        Vector3 v_i, vMin;
        double  a_i;
        double  aMin = 1e6;
        Vector3 hVec = Vector3Subtract( home, XYZ );
        
        for( uint i = 0; i < N_samples; i++ ){  
            v_i = Vector3{
                randf( -1.0,  1.0 ),
                randf( -1.0,  1.0 ),
                randf( -1.0,  1.0 )
            };
            a_i = Vector3Angle( hVec, v_i );
            if( a_i < aMin ){  vMin = v_i;  }
        }

        rtnMsg.Zb = Vector3Normalize( vMin );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( Vector3Transform( Vector3{0.0, 1.0, 0.0}, T ) , rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb, rtnMsg.Xb ) );

        return rtnMsg;
    }

    Basis consider_free_will( uint N_samples ){
        // Drunken walk: Choose a random direction in which to nudge free will
        Basis   rtnMsg;
        Vector3 vWil = Vector3{
            randf( -1.0,  1.0 ),
            randf( -1.0,  1.0 ),
            randf( -1.0,  1.0 )
        };
        rtnMsg.Zb = Vector3Normalize( vWil );
        rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( Vector3Transform( Vector3{0.0, 1.0, 0.0}, T ) , rtnMsg.Zb ) );
        rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb, rtnMsg.Xb ) );
        return rtnMsg;
    }

    double update_instincts_and_heading( const vector<boidPtr>& flock ){
        // Main navigation function
        vector<Basis> flockPoses;
        // 0. Fetch all flock poses
        for( boidPtr boid : flock ){  flockPoses.push_back( boid->get_Basis() );  }
        // FIXME, START HERE: NAVIGATION!
        // 1. Update flocking instinct
        // 2. Update home seeking instinct
        // 3. Update drunken walk
        // 4. Update where the boid is actually pointed
        // 5. Blend intincts
        // 6. Limit turn and set heading
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        build_mesh_unshared();
        build_normals_flat_unshared();
        load_mesh();
    }

};
typedef shared_ptr<Boid> boidPtr;




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){
    Matrix T;
    
}

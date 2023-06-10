// g++ 13_boids.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"

////////// TOYS ////////////////////////////////////////////////////////////////////////////////////


struct PoseMsg{
    // A message to express the position and orientation of the neighbor
    Vector3 XYZ; // World Position
	Matrix  T; //- Orientation
};


struct Basis{
    // Return message for the average bearing
    Vector3 Xb;
    Vector3 Yb;
    Vector3 Zb;
};


class Boid : public TriModel{ public:
    // A flocking entity like a bird

    /// Members ///
    float length;
    float width;
    Color sldClr; 
    float dNear;

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

    Basis consider_neighbors( const vector<PoseMsg>& nghbrPoses ){
        // Adjust bearing according to the closest neighbors in front of the boid
        Vector3 Xmean, Xnghbr;
        Vector3 Ymean, Ynghbr;
        Vector3 Zfly , diffVec;
        Basis   rtnMsg;
        uint    relevant;
        float   dist, dotFront;
        Xmean = Vector3{0.0f, 0.0f, 0.0f};
        Ymean = Vector3{0.0f, 0.0f, 0.0f};
        for( PoseMsg msg : nghbrPoses ){
            diffVec  = Vector3Subtract( msg.XYZ, XYZ );
            dist     = Vector3Length( diffVec );
            Zfly     = Vector3Transform( Vector3{0.0f, 0.0f, 1.0f}, T );
            dotFront = Vector3DotProduct( Zfly, diffVec );
            if( (dist <= dNear) || (dotFront >= 0.0f) ){
                Xnghbr = Vector3Transform( Vector3{1.0f, 0.0f, 0.0f}, msg.T );
                Ynghbr = Vector3Transform( Vector3{0.0f, 1.0f, 0.0f}, msg.T );
                Xmean  = Vector3Add( Xmean, Xnghbr );
                Ymean  = Vector3Add( Ymean, Ynghbr );
                relevant++;
            }
        }
        if( relevant ){
            Xmean = Vector3Normalize( Xmean );
            Ymean = Vector3Normalize( Ymean );
            rtnMsg.Zb = Vector3Normalize( Vector3CrossProduct( Xmean    , Ymean     ) );
            rtnMsg.Xb = Vector3Normalize( Vector3CrossProduct( Ymean    , rtnMsg.Zb ) );
            rtnMsg.Yb = Vector3Normalize( Vector3CrossProduct( rtnMsg.Zb, rtnMsg.Xb ) );
        }else{
            rtnMsg.Xb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Yb = Vector3{0.0f, 0.0f, 0.0f};
            rtnMsg.Zb = Vector3{0.0f, 0.0f, 0.0f};
        }
        return rtnMsg;
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
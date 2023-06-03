// g++ 12_synthwave-dream.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////
/// Standard ///
#include <iostream>
using std::cout, std::endl, std::ostream;
#include <algorithm> 
using std::min, std::max;
#include <set>
using std::set;
#include <map>
using std::map, std::pair;
#include <list>
using std::list;
#include <deque>
using std::deque;
#include <memory>
using std::shared_ptr;

/// Raylib ///
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
// #define RLIGHTS_IMPLEMENTATION

/// Local ///
#include "utils.hpp"
#include "rl_toybox.hpp"
// #include "rlights.h"


///// Type Aliases ///////////////////////////////
typedef unsigned short /*--*/ ushort;



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

class Plume{ public:
    // A streaking ribbon with decreasing opacity from head to tail to simulate exhaust
    // NOTE: This class assumes that backface culling is turned OFF

    /// Members ///
    uint /*--------------*/ Npairs; // -- Number of coordinate pairs allowed
    Color /*-------------*/ color; // --- Base color of the plume
    float /*-------------*/ headAlpha; // Beginning opacity
    float /*-------------*/ tailAlpha; // Ending    opacity
    deque<array<Vector3,2>> coords; // -- Ribbon data, listed from head to tail

    /// Constructors ///

    Plume( uint N, Color c, float Ah, float At ){
        Npairs    = N;
        color     = c;
        headAlpha = Ah;
        tailAlpha = At;
    }

    /// Methods ///

    void push_coord_pair( const Vector3& c1, const Vector3& c2 ){
        // Add coordinates to the head of the plume
        if( coords.size() >= Npairs )  coords.pop_back(); // If queue is full, drop the tail element
        coords.push_front(  array<Vector3,2>{ c1, c2 }  );
    }

    void draw(){
        // Render plume as a batch job
        uint    Nsize = coords.size()-1;        
        float   R     = color.r/255.0f;
        float   G     = color.g/255.0f;
        float   B     = color.b/255.0f;
        float   Aspan = headAlpha - tailAlpha;
        Vector3 c1, c2, c3, c4;
        float   A_i;
        float   A_ip1;
        
        // Begin triangle batch job
        rlBegin( RL_TRIANGLES );

        for( uint i = 0; i < Nsize; i++){
            c1    = coords[i  ][0];
            c2    = coords[i  ][1];
            c3    = coords[i+1][0];
            c4    = coords[i+1][1];
            A_i   = tailAlpha + Aspan*(Nsize-(i  ))/(1.0f*Nsize);
            A_ip1 = tailAlpha + Aspan*(Nsize-(i+1))/(1.0f*Nsize);

            if( i%2==0 ){
                /// Triangle 1: c2, c1, c3 ///
                // t1.p1 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
                // t1.p2 //
                rlVertex3f(c1.x, c1.y, c1.z);
                // t1.p3 //
                rlColor4f(R, G, B, A_ip1);
                rlVertex3f(c3.x, c3.y, c3.z);

                /// Triangle 2: c3, c4, c2 ///
                // t2.p1 //
                rlVertex3f(c3.x, c3.y, c3.z);
                // t2.p2 //
                rlVertex3f(c4.x, c4.y, c4.z);
                // t2.p3 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
            }else{
                /// Triangle 1: c1, c3, c4 ///
                // t1.p1 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c1.x, c1.y, c1.z);
                // t1.p2 //
                rlColor4f(R, G, B, A_ip1);
                rlVertex3f(c3.x, c3.y, c3.z);
                // t1.p3 //
                rlVertex3f(c4.x, c4.y, c4.z);

                /// Triangle 2: c4, c2, c1 ///
                // t2.p1 //
                rlVertex3f(c4.x, c4.y, c4.z);
                // t2.p2 //
                rlColor4f(R, G, B, A_i);
                rlVertex3f(c2.x, c2.y, c2.z);
                // t2.p3 //
                rlVertex3f(c1.x, c1.y, c1.z);
            }
        }
        // End triangle batch job
        rlEnd();
    }
};


class Icosahedron_r : public TriModel{ public:

    // ~ Constants ~
	float sqrt5 = sqrt( 5.0f ); // ----------------------------------- Square root of 5
	float phi   = ( 1.0f + sqrt5 ) * 0.5f; // ------------------------- The Golden Ratio
	float ratio = sqrt( 10.0f + ( 2.0f * sqrt5 ) ) / ( 4.0f * phi ); // ratio of edge length to radius
	
	// ~ Variables ~
	float radius;
	float a; 
	float b; 
    vvec3 V;

    // ~ Appearance ~
    Color baseClr;
    Color lineClr;

    // ~ Appearance ~
    bool  anim;
    float rolVel;
    float ptcVel;
    float yawVel;

    Icosahedron_r( float rad , const Vector3& cntr, bool active = true ) : TriModel(20){
        // Compute the vertices and faces
        XYZ    = cntr;
        radius = rad;
        a /**/ = ( radius / ratio ) * 0.5;
        b /**/ = ( radius / ratio ) / ( 2.0f * phi );

        // ~ Appearance ~
        baseClr = BLACK;
        lineClr = GOLD;

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
        load_tri( V[ 2], V[ 1], V[ 0] );
        load_tri( V[ 1], V[ 2], V[ 3] );
        load_tri( V[ 5], V[ 4], V[ 3] );
        load_tri( V[ 4], V[ 8], V[ 3] );
        load_tri( V[ 7], V[ 6], V[ 0] );
        load_tri( V[ 6], V[ 9], V[ 0] );
        load_tri( V[11], V[10], V[ 4] );
        load_tri( V[10], V[11], V[ 6] );
        load_tri( V[ 9], V[ 5], V[ 2] );
        load_tri( V[ 5], V[ 9], V[11] );
        load_tri( V[ 8], V[ 7], V[ 1] );
        load_tri( V[ 7], V[ 8], V[10] );
        load_tri( V[ 2], V[ 5], V[ 3] );
        load_tri( V[ 8], V[ 1], V[ 3] );
        load_tri( V[ 9], V[ 2], V[ 0] );
        load_tri( V[ 1], V[ 7], V[ 0] );
        load_tri( V[11], V[ 9], V[ 6] );
        load_tri( V[ 7], V[10], V[ 6] );
        load_tri( V[ 5], V[11], V[ 4] );
        load_tri( V[10], V[ 8], V[ 4] );
    }


    void update(){  rotate_RPY( rolVel, ptcVel, yawVel );  } // Rotate
    

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        build_mesh_unshared();
        // build_normals_flat_unshared();
        load_mesh();
    }

    void draw(){
        // Draw the model
        if( anim ){  update();  }
        DrawModel(      model, XYZ, 1.00, baseClr );  
        DrawModelWires( model, XYZ, 1.02, lineClr );
    }
};


class HoopTarget{ public:
    // A hoop for the player to fly through, Reminiscent of an Identity Disk from Tron

    /// Members ///
    Vector3 center;
    Vector3 normal;
    float   radInner;
    float   radOuter;
    Color   clrSolidInit;
    Color   clrSolidWin;
    Color   clrBorder;
    uint    Nseg;
    bool    scored;

    HoopTarget( Vector3 origin, Vector3 faceDirection, float ID, float OD ){
        center /*-*/ = origin;
        normal /**/  = Vector3Normalize( faceDirection ); // Enforce normal vector
        radInner     = ID/2.0f;
        radOuter     = OD/2.0f;
        clrSolidInit = SKYBLUE;
        clrSolidWin  = SKYBLUE;
        clrBorder    = RAYWHITE;
        Nseg /*---*/ = 40;
        scored /*-*/ = false;
        // Make the hoop semi-transparent until it is scored
        clrSolidInit.a = 125;
    }

    void test_segment( const Vector3& Q, const Vector3& R ){
        // Test if a line segment QR passes thru the hoop, If so then set `scored` to true
        // Author: saxbophone, https://math.stackexchange.com/a/4657621
        bool    win   = false;
        Vector3 RmQ   = Vector3Subtract( R, Q );
        float   QRlen = Vector3Length( RmQ );
        float   num   = Vector3DotProduct( RmQ, Vector3Subtract( Q, center ) );
        float   den   = Vector3DotProduct( RmQ, RmQ );
        float   tHt, dotQR;
        Vector3 G, GmQ;
        // Test for nonzero segment length
        if( den > 0.0f ){
            tHt = num / den;
            // G is closest to the hoop center on QR, BUT may NOT be between Q and R!
            G     = Vector3Subtract( Q, Vector3Scale( RmQ, tHt ) );
            GmQ   = Vector3Subtract( G, Q );
            dotQR = Vector3DotProduct( GmQ, RmQ );
            // If dot prod negative, then center is entirely "behind" segment, otherwise test within segment
            // `G` must lie within the circle
            if( dotQR > 0.0f ){  
                win = (Vector3Length( GmQ ) < QRlen) && (Vector3Distance(G,center) <= radOuter);  
            }
        }
        // If we won, then latch in the `scored` state, Let the client code unlatch if needed
        if( win )  scored = true;
    }

    void draw(){
        // Render hoop as a batch job

        Vector3 radRay = Vector3Normalize( Vector3CrossProduct( 
            normal, 
            Vector3Normalize(  Vector3{ randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0) }  )    
        ) );
        Vector3 bgnRay = radRay;
        float   incr   = 2.0f * PI / (1.0f * Nseg);
        float   margin = 0.02 * radOuter;
        Vector3 p1, p2, p3, p4, p5, p6;
        p1 = Vector3Add( center, Vector3Scale( radRay, radInner ) );
        p2 = Vector3Add( center, Vector3Scale( radRay, radOuter ) );

        // Begin triangle batch job
        rlBegin( RL_TRIANGLES );

        if( scored ) rlColor4f( clrSolidWin.r/255.0f , clrSolidWin.g/255.0f , clrSolidWin.b/255.0f , clrSolidWin.a/255.0f  );
        else /*---*/ rlColor4f( clrSolidInit.r/255.0f, clrSolidInit.g/255.0f, clrSolidInit.b/255.0f, clrSolidInit.a/255.0f );

        for( uint i = 0; i < Nseg; i++ ){

            radRay = Vector3RotateByAxisAngle( radRay, normal, incr );
            p3     = Vector3Add( center, Vector3Scale( radRay, radInner ) );
            p4     = Vector3Add( center, Vector3Scale( radRay, radOuter ) );

            /// Triangle 1: p2, p1, p3 ///
            rlVertex3f( p2.x, p2.y, p2.z );
            rlVertex3f( p1.x, p1.y, p1.z );
            rlVertex3f( p3.x, p3.y, p3.z );

            /// Triangle 2: c3, c4, c2 ///
            rlVertex3f( p3.x, p3.y, p3.z );
            rlVertex3f( p4.x, p4.y, p4.z );
            rlVertex3f( p2.x, p2.y, p2.z );

            p1 = p3;
            p2 = p4;
        }

        // End triangle batch job
        rlEnd();
        
        // Begin border batch job
        radRay = bgnRay;
        p1 = Vector3Add( center, Vector3Scale( radRay, radInner-margin ) );
        p2 = Vector3Add( center, Vector3Scale( radRay, radOuter+margin ) );
        p5 = Vector3Add( center, Vector3Scale( radRay, radOuter+2.0*margin ) );
        
        rlBegin( RL_LINES );

        rlColor4f( clrBorder.r/255.0f, clrBorder.g/255.0f, clrBorder.b/255.0f, clrBorder.a/255.0f );

        for( uint i = 0; i < Nseg; i++ ){

            radRay = Vector3RotateByAxisAngle( radRay, normal, incr );
            p3     = Vector3Add( center, Vector3Scale( radRay, radInner-margin ) );
            p4     = Vector3Add( center, Vector3Scale( radRay, radOuter+margin ) );
            p6     = Vector3Add( center, Vector3Scale( radRay, radOuter+2.0*margin ) );

            /// Segment 1: p1-to-p3 ///
            rlVertex3f( p1.x, p1.y, p1.z );
            rlVertex3f( p3.x, p3.y, p3.z );

            /// Segment 2: p2-to-p4 ///
            rlVertex3f( p2.x, p2.y, p2.z );
            rlVertex3f( p4.x, p4.y, p4.z );

            /// Segment 3: p5-to-p6 ///
            rlVertex3f( p5.x, p5.y, p5.z );
            rlVertex3f( p6.x, p6.y, p6.z );

            p1 = p3;
            p2 = p4;
            p5 = p6;
        }

        // End border batch job
        rlEnd();
    }
};


////////// TERRAIN /////////////////////////////////////////////////////////////////////////////////

enum NEIGHBORS{
    X_POS,  X_NEG,
    Y_POS,  Y_NEG,
};
NEIGHBORS NEIGHBORHOOD[4] = {  X_POS,  X_NEG,  Y_POS,  Y_NEG  };


////////// TerrainTile ////////////////////////////////////////////////////

class TerrainTile : public TriModel { public:
    // Rectangular plate of randomized terrain

    /// Member Data ///

    vector<vector<Vector3>> pts; // -- Grid points in 3D space
    float   scl; // -- Scale of each cell
    ulong   M; // ---- Number of rows
    ulong   N; // ---- Number of cells per row
    Color   gndClr; // Triangle fill color
    Color   linClr; // Triangle line color
    float   offset; // Z bump for lines
    Vector3 posn1; //- Facet drawing origin
    float   pScale; // Perlin scale param
    bool    loaded; // Flag: Is this tile ready to draw?
    float   loX;
    float   hiX;
    float   loY;
    float   hiY;

    /// Child Objects ///

    bool /*------------------------*/ icosGen; // --- Whether or not to generate icosahedra
    vector<shared_ptr<Icosahedron_r>> props; // ----- Icosahedra contained in this tile
    bool /*------------------------*/ hoopGen; // --- Whether or not to generate hoops
    vector<shared_ptr<HoopTarget>>    hoops; // ----- Hoops contained in this tile
    bool /*------------------------*/ scoreTest; // - Whether to check hoop scoring for the player
    bool /*------------------------*/ runPlyrChks; // Whether or not to run PvE checks

    /// Constructors & Helpers ///

    void gen_heightmap_uniform_random(){
        // Generate a heightmap with uniform random sampling
        vvec3 row;
        for( ulong i = 0; i < M; i++ ){
            for( ulong j = 0; j < N; j++ ){
                row.push_back(  Vector3{ 
                    posn1.x + j*scl + randf( -scl*0.25, +scl*0.25 ), 
                    posn1.y + i*scl + randf( -scl*0.25, +scl*0.25 ),
                    posn1.z + randf()*scl 
                }  );
            }
            pts.push_back( row );
            row.clear();
        }
    }

    void gen_heightmap_perlin( float pScale = 1.6f ){
        // Create a Perlin Image
        // Calc corners
        float factor =   5.0f;
        int   hI     = 3*M; // Height of the image
        int   wI     = 3*N; // Width  of the image
        ulong rowUL  = randi( 0, hI - M );
        ulong colUL  = randi( 0, wI - N );
        ulong rowLR  = rowUL + M;
        ulong colLR  = colUL + N;
        // Select subimage
        Image  perlinImage = GenImagePerlinNoise( hI, wI, 0, 0, pScale ); 
        Color* perlinClrs  = LoadImageColors( perlinImage );
        vvf    zValues     = get_subimage_red_intensity( perlinClrs, perlinImage.width, rowUL, colUL, rowLR, colLR );
        // Generate heightmap
        vvec3 row;
        float accum = 0;
        float elem;
        for( ulong i = 0; i < M; i++ ){
            row.clear();
            for( ulong j = 0; j < N; j++ ){
                elem = zValues[i][j]*scl*factor;
                accum += elem;
                row.push_back(  Vector3{ 
                    posn1.x + j*scl + randf( -scl*0.25, +scl*0.25 ), 
                    posn1.y + i*scl + randf( -scl*0.25, +scl*0.25 ),
                    posn1.z + elem
                }  );
            }
            pts.push_back( row );
        }
        accum /= M*N;
        for( ulong i = 0; i < M; i++ ){
            for( ulong j = 0; j < N; j++ ){
                pts[i][j].z -= accum;
            }
        }
        if( perlinClrs )  delete perlinClrs;
        UnloadImage( perlinImage );  

        // zValues.clear();
    }

    void build_triangles(){
        // Turn the heightmap into a mesh
        for( ulong i = 0; i < M-1; i++ ){
            for( ulong j = 0; j < N-1; j++ ){
                Vector3 v1, v2, v3, v4;
                // Load points
                v1 = pts[i  ][j  ];
                v2 = pts[i  ][j+1];
                v3 = pts[i+1][j  ];
                v4 = pts[i+1][j+1];
                // Randomize cross right or cross left
                if( randf() < 0.5f ){
                    load_tri( v1, v2, v3 );
                    load_tri( v3, v2, v4 );
                }else{
                    load_tri( v1, v2, v4 );
                    load_tri( v3, v1, v4 );
                }
            }
        }
    }

    TerrainTile( float scale, ulong Mrows, ulong Ncols, Vector3 origin,
                 bool populateProps = true ) : TriModel( (Mrows-1)*(Ncols-1)*2 ){
        // Generate points and load triangles
        
        // cout << "Basic `TerrainTile` constructor!" << endl;

        // 0. Init
        M /*-*/ = Mrows;
        N /*-*/ = Ncols;
        scl     = scale;
        offset  = scale/200.0f;
        gndClr  = BLACK; 
        linClr  = MAGENTA;
        posn1   = origin; 
        pScale  = 10.0f;
        visible = true;
        loaded  = false;
        icosGen = populateProps;
        hoopGen = populateProps;
        loX     =  1e6;
        hiX     = -1e6;
        loY     =  1e6;
        hiY     = -1e6;

        // 1. Generate points
        gen_heightmap_perlin( pScale );

        // 2. Generate icos
        float loZ     =  0.0;
        float hiZ     =  0.0;
        vvec3 corners = get_corners();
        if( icosGen ){
            uint  icosMax =  6;
            uint  Ncreate = randi( 0, icosMax );
            float loRad   =  5.0f;
            float hiRad   = 50.0f;
            for( Vector3 corner: corners ){  
                loZ += corner.z;  
                if( corner.x < loX )  loX = corner.x;
                if( corner.x > hiX )  hiX = corner.x;
                if( corner.y < loY )  loY = corner.y;
                if( corner.y > hiY )  hiY = corner.y;
            }
            loZ /= 4.0f;
            hiZ = loZ + 200.0f;
            for( uint i = 0; i < Ncreate; i++ ){
                props.push_back( shared_ptr<Icosahedron_r>( new Icosahedron_r{
                    randf( loRad, hiRad ),
                    Vector3{ randf( loX, hiX ), randf( loY, hiY ), randf( loZ, hiZ ) }
                } ) );
            }
        }

        if( hoopGen ){
            uint  hoopMax =  4;
            uint  Ncreate = randi( 0, hoopMax );
            float loRad   = 10.0f;
            float hiRad   = 30.0f;
            float diffRad = -5.0f;
            float rad;
            for( uint i = 0; i < Ncreate; i++ ){
                rad = randf( loRad, hiRad );
                hoops.push_back( shared_ptr<HoopTarget>( new HoopTarget(
                    Vector3{ randf( loX, hiX ), randf( loY, hiY ), randf( loZ, hiZ ) },
                    Vector3Normalize( Vector3{randf( -1.0f, 1.0f ), randf( -1.0f, 1.0f ), 0.0f} ),
                    rad+diffRad, rad
                ) ) );
            }
        }

    }

    ~TerrainTile(){
        // Free all allocated memory
        // UnloadModel( model );  
        props.clear();
        hoops.clear();
    }

    void stitch_X_POS_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong i = 0; i < M; i++ ){  pts[i][0] = Vector3{ OtherPlate.pts[i][N-1] };  }
        // 3. Smooth
        for( ulong i = 0; i < M; i++ ){  
            z1 = pts[i][2].z;
            z2 = pts[i][0].z;
            pts[i][1].z = randf_bn( z1, z2 );
        }
    }

    void stitch_X_NEG_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong i = 0; i < M; i++ ){  pts[i][N-1] = Vector3{ OtherPlate.pts[i][0] };  }
        // 3. Smooth
        for( ulong i = 0; i < M; i++ ){  
            z1 = pts[i][N-3].z;
            z2 = pts[i][N-1].z;
            pts[i][N-2].z = randf_bn( z1, z2 );
        }
    }

    void stitch_Y_POS_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong j = 0; j < N; j++ ){  pts[0][j] = Vector3{ OtherPlate.pts[M-1][j] };  }
        // 3. Smooth
        for( ulong j = 0; j < N; j++ ){  
            z1 = pts[2][j].z;
            z2 = pts[0][j].z;
            pts[1][j].z = randf_bn( z1, z2 );
        }
    }

    void stitch_Y_NEG_of( const TerrainTile& OtherPlate ){
        float z1, z2;
        // 2. Stitch
        for( ulong j = 0; j < N; j++ ){  pts[M-1][j] = Vector3{ OtherPlate.pts[0][j] };   }      
        // 3. Smooth
        for( ulong j = 0; j < N; j++ ){  
            z1 = pts[M-3][j].z;
            z2 = pts[M-1][j].z;
            pts[M-2][j].z = randf_bn( z1, z2 );
        }          
    }

    /// Methods ///

    float get_greatest_elevation(){
        // Get the highest point in the terrain
        float top = -1000.0f;
        for( ulong i = 0; i < M; i++ ){
            for( ulong j = 0; j < N; j++ ){
                if( pts[i][j].z > top )  top = pts[i][j].z;
            }
        }
        return top;
    }

    vvec3 get_corners() const{
        // Get the positions of the corners of the tile
        vvec3 corners;
        // cout << "About to check size ..." << endl;
        // cout << "How many points?: " << pts.size() << endl;
        if( pts.size() ){
            // cout << "There are corners to get ..." << endl;
            corners.push_back(  pts[0  ][0  ]  );
            corners.push_back(  pts[M-1][0  ]  );
            corners.push_back(  pts[M-1][N-1]  );
            corners.push_back(  pts[0  ][N-1]  );
        }
        return corners;
    }

    bool p_point_within_XY_bounds( const Vector3& query ){
        // Return true if the point is within the tile when both are projected to the XY plane
        return ((loX <= query.x) && (query.x <= hiX) && (loY <= query.y) && (query.y <= hiY));
    }

    void check_player_relevant( const Vector3& query ){
        // Determine whether or not to run PvE checks
        if( p_point_within_XY_bounds( query ) )
            runPlyrChks = true;
        else
            runPlyrChks = false;
    }

    ///// Rendering //////////////////////////////
    // WARNING: Requires window init to call!

    void load_geo(){
        // Get the model ready for drawing
        // cout << "\t\t`build_triangles` ..." << endl;
        build_triangles();
        // cout << "\t\t`build_mesh_unshared` ..." << endl;
        build_mesh_unshared();
        // cout << "\t\t`build_normals_flat_unshared` ..." << endl;
        // build_normals_flat_unshared();
        // cout << "\t\t`load_mesh` ..." << endl;
        load_mesh();
        if( icosGen ){  for( shared_ptr<Icosahedron_r> prop : props ){  
            prop->load_geo();
        }  }
        loaded = true;
    }

    void draw(){
        // Draw facets, shift up, draw lines
        DrawModel(      model, Vector3{ 0.0f, 0.0f, 0.0f   }, 1.0, gndClr );  
        DrawModelWires( model, Vector3{ 0.0f, 0.0f, offset }, 1.0, linClr );
        if( icosGen ){  for( shared_ptr<Icosahedron_r> prop : props ){  prop->draw();  }  }
        if( hoopGen ){  for( shared_ptr<HoopTarget>    hoop : hoops ){  hoop->draw();  }  }
    }
};



////////// TerrainGrid ////////////////////////////////////////////////////

// FIXME, START HERE: QUERY ALL THE HOOPS IN THE ACTIVE TILE FOR SCORING

class TerrainGrid{ public:
    // Managing structure for multiple `TerrainTile`s

    /// Members ///
    float sclCell;
    ulong MrowsTile;
    ulong NcolsTile;

    /// Child Objects ///
    map<array<int,2>,shared_ptr<TerrainTile>> tiles;

    /// Constructors & Destructors ///

    TerrainGrid( float cellScale, ulong Mrows, ulong Nrows ){
        // Create a new grid with the center tile populated
        // Set params
        sclCell   = cellScale;
        MrowsTile = Mrows;
        NcolsTile = Nrows;
        // Populate center tile
        tiles[ {0,0} ] = shared_ptr<TerrainTile>(  
            new TerrainTile{ sclCell, MrowsTile, NcolsTile, Vector3{ 0.0f, 0.0f, 0.0f } }  
        );
    }

    ~TerrainGrid(){
        // Delete all tiles
        // for( pair<array<int,2>,shared_ptr<TerrainTile>> elem : tiles ){  delete elem.second;  }
        tiles.clear();
    }

    /// Methods ///

    bool p_cell_occupied( const array<int,2>& addr ){  return (tiles.count( addr ) > 0);  } // Is the cell at `addr` occupied?

    vector<array<int,2>> neighbors_of( const array<int,2>& addr ){
        // Return the Von Neumann neighborhood of `addr`
        vector<array<int,2>> rtnLst;
        rtnLst.push_back( { addr[0]  , addr[1]+1 } ); // X_POS
        rtnLst.push_back( { addr[0]  , addr[1]-1 } ); // X_NEG
        rtnLst.push_back( { addr[0]+1, addr[1]   } ); // Y_POS
        rtnLst.push_back( { addr[0]-1, addr[1]   } ); // Y_NEG
        return rtnLst;
    }

    Vector3 get_tile_origin( const array<int,2>& addr ){
        // Get the origin of the `addr` in 3D space
        return Vector3{
            1.0f*addr[0]*(1.0f*NcolsTile-1)*sclCell,
            1.0f*addr[1]*(1.0f*MrowsTile-1)*sclCell,
            0.0f 
        };
    }

    vvec3 get_tile_corners( const array<int,2>& addr ){
        // Get the positions of the corners of the tile at the `addr`
        vvec3   rtnLst;
        Vector3 origin = get_tile_origin( addr );
        rtnLst.push_back( origin );
        rtnLst.push_back( Vector3Add(  origin, Vector3{ (1.0f*NcolsTile-1)*sclCell,  0.0f                     , 0.0f }  ) );
        rtnLst.push_back( Vector3Add(  origin, Vector3{  0.0f                     , (1.0f*MrowsTile-1)*sclCell, 0.0f }  ) );
        rtnLst.push_back( Vector3Add(  origin, Vector3{ (1.0f*NcolsTile-1)*sclCell, (1.0f*MrowsTile-1)*sclCell, 0.0f }  ) );
        return rtnLst;
    }

    void populate_neighbors_of( const array<int,2>& addr ){
        // Create the Von Neumann neighborhood of `addr`
        shared_ptr<TerrainTile> nuTile    = nullptr;
        vector<array<int,2>>    neighbors = neighbors_of( addr ); // {X_POS, X_NEG, Y_POS, Y_NEG}
        vector<array<int,2>>    nghbrNghbrs;
        int /*---------------*/ i;
        
        for( array<int,2> nghbrAddr : neighbors ){
            if( !p_cell_occupied( nghbrAddr ) ){
                nuTile = shared_ptr<TerrainTile>( 
                    new TerrainTile{ sclCell, MrowsTile, NcolsTile, get_tile_origin( nghbrAddr ) }
                );
                tiles[ nghbrAddr ] = nuTile;
                nghbrNghbrs = neighbors_of( nghbrAddr ); // {X_POS, X_NEG, Y_POS, Y_NEG}
                i = 0;
                for( array<int,2> nghbrNghbr : nghbrNghbrs ){
                    if( p_cell_occupied( nghbrNghbr ) ){
                        switch (i){
                            case 0:
                                nuTile->stitch_Y_NEG_of( *tiles[ nghbrNghbr ] );
                                break;
                            case 1:
                                nuTile->stitch_Y_POS_of( *tiles[ nghbrNghbr ] );
                                break;
                            case 2:
                                nuTile->stitch_X_NEG_of( *tiles[ nghbrNghbr ] );
                                break;
                            case 3:
                                nuTile->stitch_X_POS_of( *tiles[ nghbrNghbr ] );
                                break;
                            default:
                                break;
                        }
                    }
                    i++;
                }
            }
        }
    }

    array<int,2> p_visible_and_oncoming( const FlightFollowThirdP_Camera& cam, const array<int,2>& addr ){
        // Return a two-part flag [ <Tile is close to the camera>, <Tile is in view or is in danger of coming into view> ]
        array<int,2> rtnFlags = {0,0};
        vvec3 /*--*/ corners = get_tile_corners( addr );
        float /*--*/ dist, 
        /*--------*/ xMin =  10000.0f, 
        /*--------*/ xMax = -10000.0f, 
        /*--------*/ yMin =  10000.0f, 
        /*--------*/ yMax = -10000.0f;
        for( Vector3 corner : corners ){
            // Determine Visible
            if( Vector3Distance( cam.position, corner ) <= (1.5f*cam.dDrawMax) )  rtnFlags[0] = 1;
            // Determine oncoming
            dist = cam.signed_distance_to_frustrum( corner );
            if( (!isnan( dist )) && (dist <= (1.5f*cam.dDrawMax)) )  rtnFlags[1] = 1;
            if( corner.x < xMin )  xMin = corner.x;
            if( corner.x > xMax )  xMax = corner.x;
            if( corner.y < yMin )  yMin = corner.y;
            if( corner.y > yMax )  yMax = corner.y;
        }
        if(  (cam.position.x >= xMin) && (cam.position.x <= xMax) && (cam.position.y >= yMin) && (cam.position.y <= yMax)  ){
            rtnFlags[0] = 1;
            rtnFlags[1] = 1;
        }
        return rtnFlags;
    }

    vector<array<int,2>> mark_visible_and_return_oncoming( const FlightFollowThirdP_Camera& cam ){
        // Mark tiles in/visible, Return a list of oncoming tiles that might need expansion
        array<int,2> /*------*/ currAddr;
        shared_ptr<TerrainTile> currTile = nullptr;
        array<int,2> /*------*/ tileRslt;
        vector<array<int,2>>    rtnLst;

        // For every tile
        for( pair<array<int,2>,shared_ptr<TerrainTile>> elem : tiles ){
            currAddr = elem.first;
            currTile = elem.second;
            tileRslt = p_visible_and_oncoming( cam, currAddr );
            if( tileRslt[0] ){  currTile->visible = true;  }else{  currTile->visible = false;  }
            if( tileRslt[1] ){
                rtnLst.push_back( currAddr );
            }
            if( (currTile->visible) && (!(currTile->loaded)) ){
                currTile->load_geo();
                // currTile->loaded = true;
            }
        }
        return rtnLst;
    }

    void mark_visible_and_expand_border( const FlightFollowThirdP_Camera& cam ){
        // Generate tiles as we approach them and maintain their visibility
        vector<array<int,2>> borderTiles = mark_visible_and_return_oncoming( cam );
        for( array<int,2> borderAddr : borderTiles ){
            populate_neighbors_of( borderAddr );
        }
    }

    void load_geo(){
        // Load geometry for all existing tiles
        for( pair<array<int,2>,shared_ptr<TerrainTile>> elem : tiles ){  elem.second->load_geo();  }
    }

    void draw(){
        // Load geometry for all existing tiles
        for( pair<array<int,2>,shared_ptr<TerrainTile>> elem : tiles ){  
            if( (elem.second->visible) && (elem.second->loaded) )  elem.second->draw();  
        }
    }
};



////////// MAIN ////////////////////////////////////////////////////////////////////////////////////

int main(){

    rand_seed();

    const Vector2 res = { 1200, 600 };
    ulong Mrows = 50,
    /*-*/ Nrows = 50;
    float tCellScale = 10.0f,
    /*-*/ wingspan   = 10.0f;

    /// Scene Init: Pre-Window ///

    TerrainGrid terrainTiles{ tCellScale, Mrows, Nrows};
    terrainTiles.populate_neighbors_of( {0,0} );

    // TerrainTile terrain{ tCellScale, Mrows, Nrows };
    cout << "Terrain built!" << endl;
    
    DeltaGlider  glider{ wingspan };
	float /*--*/ frameRotateRad = 3.1416/120.0;
    float /*--*/ frameThrust    = 36.0/60.0;
    vvec3 /*--*/ gliderPoints;
    Vector3 /**/ vec1, vec2;

    Plume plume{ 60, ORANGE, 1.0f, 0.0f };
    vvec3 skltn;

    /// Window Init ///
    InitWindow( (int) res.x, (int) res.y, "Terrain Gen + Glider + Bloom Shader" );
    SetTargetFPS( 60 );
    rlEnableSmoothLines();
    rlDisableBackfaceCulling();

    /// Scene Init: Post-Window ///
    cout << "About to load geo ..." << endl;
    glider.load_geo();
    cout << "\tGlider loaded!" << endl;
    terrainTiles.load_geo();
    cout << "\tTerrain loaded!" << endl;
    // leftVortex.load_geo();
    cout << "Geo loaded!" << endl;
    glider.set_XYZ( 
        Nrows*tCellScale/2.0f, 
        Nrows*tCellScale/2.0f, 
        terrainTiles.tiles[{0,0}]->get_greatest_elevation()+10.0f
    );
    glider.rotate_RPY( 0.0, 3.1416/2.0, 3.1416 );

    FlightFollowThirdP_Camera camera{
        35.0,
		glider.get_XYZ(),
		glider.T
    };

    
    ////////// Shader Init: Pre-Window /////////////////////////////////////////////////////////////

    // bloom shader
    Shader bloom = LoadShader( 0, "shaders/bloom.fs" );
    // bloom.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(bloom, "matModel");

    RenderTexture2D target = LoadRenderTexture( res.x, res.y );



    ////////// RENDER LOOP /////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// UPDATE PHASE /////////////////////////

        // Keyboard input
        if( IsKeyDown( KEY_Z     ) ){  glider.rotate_RPY( -frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_X     ) ){  glider.rotate_RPY(  frameRotateRad,  0.0           ,  0.0            );  }
		if( IsKeyDown( KEY_UP    ) ){  glider.rotate_RPY(  0.0           ,  frameRotateRad,  0.0            );  }
		if( IsKeyDown( KEY_DOWN  ) ){  glider.rotate_RPY(  0.0           , -frameRotateRad,  0.0            );  }
        if( IsKeyDown( KEY_LEFT  ) ){  glider.rotate_RPY(  0.0           ,  0.0           ,  frameRotateRad );  }
		if( IsKeyDown( KEY_RIGHT ) ){  glider.rotate_RPY(  0.0           ,  0.0           , -frameRotateRad );  }

		// gamepad input
		if( IsGamepadAvailable(0) ){
            glider.rotate_RPY( 
                 frameRotateRad*GetGamepadAxisMovement( 0, GAMEPAD_AXIS_RIGHT_X ),  
                 0.0,  
                 0.0            
            );
            glider.rotate_RPY( 
                 0.0, 
                -frameRotateRad*GetGamepadAxisMovement( 0, GAMEPAD_AXIS_RIGHT_Y ), 
                 0.0 
            );
			glider.rotate_RPY( 
                0.0, 
                0.0, 
                -frameRotateRad*GetGamepadAxisMovement( 0, GAMEPAD_AXIS_LEFT_X ) 
            );
		}

        glider.z_thrust( frameThrust );
        
        skltn = glider.get_skeleton_points_in_world_frame();
        vec1  = Vector3Subtract( skltn[1], skltn[0] );
        vec2  = Vector3Subtract( skltn[2], skltn[0] );
        vec1  = Vector3Add( Vector3Scale( vec1, 1.0f/Vector3Length(vec1)*wingspan*0.125 ), skltn[0] );
        vec2  = Vector3Add( Vector3Scale( vec2, 1.0f/Vector3Length(vec2)*wingspan*0.125 ), skltn[0] );
        plume.push_coord_pair( vec1, vec2 );

        camera.update_target_position( glider.get_XYZ() );
		camera.advance_camera();
        terrainTiles.mark_visible_and_expand_border( camera );

        ///// DRAW PHASE /////////////////////////
        
        BeginTextureMode( target ); // - Enable drawing to texture
            ClearBackground( BLACK ); // Clear texture background
            BeginMode3D( camera ); // -- Begin 3d mode drawing
                terrainTiles.draw();
                glider.draw();
                plume.draw();
                // terrainTiles.draw();
            EndMode3D(); //- End 3d mode drawing, returns to orthographic 2d mode
        EndTextureMode(); // End drawing to texture (now we have a texture available for next passes)
        
        BeginDrawing();
            ClearBackground( BLACK ); // Clear screen background

            // Render generated texture using selected postprocessing shader
            BeginShaderMode( bloom );
                // NOTE: Render texture must be y-flipped due to default OpenGL coordinates (left-bottom)
                DrawTextureRec(
                    target.texture, 
                    (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, 
                    (Vector2){ 0, 0 }, 
                    BLACK // WHITE 
                );
            EndShaderMode();
            DrawFPS( 10, 10 );
        EndDrawing();
    }



    ////////// CLEANUP /////////////////////////////////////////////////////////////////////////////

    // UnloadShader( bloom );
    // UnloadRenderTexture( target );    
    // UnloadModel( glider.model  );
    CloseWindow(); // Close window and OpenGL context

    return 0;
}
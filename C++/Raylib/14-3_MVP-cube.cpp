// g++ 14-3_MVP-cube.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <deque>
using std::deque;
#include <array>
using std::array;
#include <vector>
using std::vector;

/// Raylib ///
#include "raylib.h"
#include "raymath.h"


////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

struct DynaCube{
    // A simple cube with optionally-drawn facets

    /// Members ///
    deque<array<Vector3,3>> triangles; // - Dynamic geometry, each array is a facet of 3x vertices
    deque<array<Vector3,3>> normals; // --- Normal vector for each facet
    deque<array<Color,3>>   facetColors; // Vertex colors for each facet
    deque<bool> /*-------*/ activeTri; // - Flags for whether to draw each triangle
    Mesh /*--------------*/ mesh; // ------ Raylib mesh geometry
	Model /*-------------*/ model; // ----- Raylib drawable model
    size_t /*------------*/ Nfrms; // ----- Frame divisor for state change
    size_t /*------------*/ iFrms; // ----- Age of cube in frames

    /// Constructor ///
    DynaCube( float sideLen ){
        // Create triangles that make up the cube
        // 0. Init
        array<Vector3,3> pushArr;
        array<Vector3,3> pushNrm;
        array<Color,3>   pushClr;
        vector<Vector3>  V;
        float /*------*/ halfLen = sideLen/2.0;
        Color /*------*/ R{ 255,   0,   0, 255 };
        Color /*------*/ G{   0, 255,   0, 255 };
        Color /*------*/ B{   0,   0, 255, 255 };
        // 1. Establish vertices
        V.push_back( Vector3{ -halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{ -halfLen,  halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen, -halfLen,  halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen, -halfLen } );
        V.push_back( Vector3{  halfLen,  halfLen,  halfLen } );
        // 2. Build triangles
        pushArr = { V[0], V[3], V[2] };  triangles.push_back( pushArr );
        pushArr = { V[0], V[1], V[3] };  triangles.push_back( pushArr );
        pushArr = { V[6], V[4], V[0] };  triangles.push_back( pushArr );
        pushArr = { V[6], V[0], V[2] };  triangles.push_back( pushArr );
        pushArr = { V[0], V[4], V[5] };  triangles.push_back( pushArr );
        pushArr = { V[0], V[5], V[1] };  triangles.push_back( pushArr );
        pushArr = { V[7], V[6], V[2] };  triangles.push_back( pushArr );
        pushArr = { V[7], V[2], V[3] };  triangles.push_back( pushArr );
        pushArr = { V[4], V[6], V[7] };  triangles.push_back( pushArr );
        pushArr = { V[4], V[7], V[5] };  triangles.push_back( pushArr );
        pushArr = { V[1], V[5], V[7] };  triangles.push_back( pushArr );
        pushArr = { V[1], V[7], V[3] };  triangles.push_back( pushArr );
        // 3. Build Colors
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        pushClr = { R, G, B };  facetColors.push_back( pushClr );
        pushClr = { B, R, G };  facetColors.push_back( pushClr );
        pushClr = { G, B, R };  facetColors.push_back( pushClr );
        // 4. Init mesh
        mesh = Mesh{};
        // FIXME, START HERE: ALLOCATE MEM FOR VERTICES, INDICES, NORMALS, AND COLORS
    }
};
// g++ 14-3_MVP-cube.cpp -std=c++17 -lraylib

////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Standard ///
#include <deque>
using std::deque;
#include <array>
using std::array;

/// Raylib ///
#include "raylib.h"
#include "raymath.h"

////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

struct DynaCube{
    // A simple cube with optionally-drawn facets

    /// Members ///
    deque<array<Vector3,3>> triangles; // - Dynamic geometry
    deque<array<Color,3>>   facetColors; // Vertex colors for each facet
    deque<bool> /*-------*/ activeTri; // - Flags for whether to draw each triangle
    Mesh /*--------------*/ mesh; // ------ Raylib mesh geometry
	Model /*-------------*/ model; // ----- Raylib drawable model

    /// Constructor ///
    DynaCube( float sideLen ){
        // Create triangles that make up the cube
        array<Vector3,3> pushArr;
        float /*------*/ halfLen = sideLen/2.0;
    }
};
// g++ 24_cubelings.cpp -std=c++17 -lraylib -O3
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"



////////// TOYS ////////////////////////////////////////////////////////////////////////////////////

struct XY_Grid{
    Vector3 cntr;
    float   xLen;
    float   yLen;
    float   unit;
    Color   colr;
};  

class CompositeModel{ public:
    // Contains multiple `DynaMesh` parts

    /// Members ///

    Matrix /*----*/ xfrm;
    vector<dynaPtr> parts;

    /// Methods ///

    void set_shader( Shader shader ){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->set_shader( shader );  }
    }

    void set_position( const Vector3& posn ){
        // Set the position of the `Cubeling`
        xfrm = set_posn( xfrm, posn );
    }

    void set_part_poses(){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->transform_from_parent( xfrm );  }
    }

    void draw(){
        // Set the shader for all parts
        for( dynaPtr& part : parts ){  part->draw();  }
    }
};
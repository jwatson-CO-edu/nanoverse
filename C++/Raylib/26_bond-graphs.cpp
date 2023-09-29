// g++ 24_cubelings.cpp -std=c++17 -lraylib -O3
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"

///// Aliases ////////////////////////////////////
typedef array<float,2> eff_flo;


////////// BOND GRAPH COMPONENTS ///////////////////////////////////////////////////////////////////

///// Forward Declarations ///////////////////////

class  Node_BG;  typedef shared_ptr<Node_BG> nodePtr;
struct Edge_BG;  typedef shared_ptr<Edge_BG> edgePtr;

///// 1-Port Node Laws ///////////////////////////

eff_flo error_energy(){  return {nanf(""), nanf("")};  }

class Law_Node{ public:
    // Base class for all node Constitutive Laws
    virtual eff_flo operator()( nodePtr node ){  return error_energy();  }
}; 
typedef shared_ptr<Law_Node> lawNodPtr;

class Inertia_Law : public Law_Node{ public:
    // Inertias store kinetic energy as a function of momentum
    eff_flo operator()( nodePtr node ){  return {
    /* e = */ (node->I)*node->df, 
    /* f = */ (1.0f/node->I)*node->p 
    };  }
};

class Capacitor_Law : public Law_Node{ public:
    // Capacitors store potential energy as a function of displacement
    eff_flo operator()( nodePtr node ){  return {
    /* e = */ (1.0f/node->C)*node->q, 
    /* f = */ (node->C)*node->de, 
    };  }
};

class Resistor_Law : public Law_Node{ public:
    // A resistors is a loss mechanism that dissipates energy
    eff_flo operator()( nodePtr node ){  return {
    /* e = */ (node->R)*(node->f), 
    /* f = */ (1.0f/node->R)*node->e
    };  }
};

///// Nodes //////////////////////////////////////

enum Node_Type{

    /// 1-Port ///
    INERTIA,
    CAPACITR,
    RESISTOR,
    SRC_EFRT, // Effort Source
    SRC_FLOW, // Flow   Source

    /// 2-Port ///
    GYRATOR,
    XFORMER,

    /// N-Port ///
    JUNCTN_0, // Type 0 Junction
    JUNCTN_1, // Type 1 Junction

};

enum Flow_Dir{
    INPUT,
    OUTPUT,
};

class Node_BG{ public:
    // Base class for all Bond Graph nodes

    /// Members ///
    Node_Type /*--*/ type; // ---- Type of node
    string /*-----*/ name; // ---- Node label
    vector<edgePtr>  ports; // --- 1, 2, or N
    vector<Flow_Dir> flowsDirs; // Direction of each flow (Not used?)
    ubyte /*------*/ portLim; // - Limit on number of ports, (0 = no limit)
    lawNodPtr /*--*/ consLaw; // - Constitutive Law

    // Energy Content //
    float  e; // effort
    float de; // Time rate change in effort
    float  f; // flow
    float df; // Time rate change in flow

    // Constants //
    float I; // inertia
    float C; // capacitance
    float R; // resistance

    // Quantities //
    float p; // momentum
    float q; // displacement

    // 2-Port Nodes //
};

struct Edge_BG{
    // A pipe for effort and flow

    /// End 1 ///
    nodePtr end1;
    float   e1;
    float   f1;

    /// End 2 ///
    nodePtr end2;
    float   e2;
    float   f2;
};



////////// BOND GRAPH VISUALIZATION ////////////////////////////////////////////////////////////////


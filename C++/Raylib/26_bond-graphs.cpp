// g++ 24_cubelings.cpp -std=c++17 -lraylib -O3
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"

///// Aliases ////////////////////////////////////
typedef array<float,2> eff_flo;


////////// SIMULATION //////////////////////////////////////////////////////////////////////////////

///// Forward Declarations ///////////////////////

struct Port_1_Node;  typedef shared_ptr<Port_1_Node> prt1NodePtr;
struct Edge_BG; /**/ typedef shared_ptr<Edge_BG>     edgePtr;

///// 1-Port Node Laws ///////////////////////////

eff_flo error_energy(){  return {nanf(""), nanf("")};  }

class Law_Port_1_Node{ public:
    virtual eff_flo operator()( prt1NodePtr node ){  return error_energy();  }
}; 
typedef shared_ptr<Law_Port_1_Node> lawNodPtr;

class Inertia_Law : public Law_Port_1_Node{ public:
    // Inertias store kinetic energy as a function of momentum
    eff_flo operator()( prt1NodePtr node ){  return {
    /* e = */ (node->I)*node->df, 
    /* f = */ (1.0f/node->I)*node->p 
    };  }
};

class Capacitor_Law : public Law_Port_1_Node{ public:
    // Capacitors store potential energy as a function of displacement
    eff_flo operator()( prt1NodePtr node ){  return {
    /* e = */ (1.0f/node->C)*node->q, 
    /* f = */ (node->C)*node->de, 
    };  }
};

class Resistor_Law : public Law_Port_1_Node{ public:
    // A resistors is a loss mechanism that dissipates energy
    eff_flo operator()( prt1NodePtr node ){  return {
    /* e = */ (node->R)*(node->f), 
    /* f = */ (1.0f/node->R)*node->e
    };  }
};

///// Nodes //////////////////////////////////////

enum Port_1_Type{
    INERTIA,
    CAPACITR,
    RESISTOR,
    SRC_EFRT, // Effort Source
    SRC_FLOW, // Flow   Source
    JUNCTN_0, // Type 0 Junction
    JUNCTN_1, // Type 1 Junction
};

struct Port_1_Node{

    /// Members ///
    Port_1_Type type;
    edgePtr     port; // Connection to the system

    // Energy Content //
    float  e; // - effort
    float de; // - Time rate change in effort
    float  f; // - flow
    float df; // - Time rate change in flow

    // Constants //
    float I; // -- inertia
    float C; // -- capacitance
    float R; // -- resistance

    // Quantities //
    float p; // -- momentum
    float q; // -- displacement

    // 2-Port Nodes //
};

enum Edge_Type{

};

struct Edge_BG{
    Edge_Type type;
};
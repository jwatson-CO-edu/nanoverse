// g++ 26_bond-graphs.cpp -std=c++17 -lraylib -O3 -o bondGraphs.out
// Recreate endearing block creatures from graphics class


////////// INIT ////////////////////////////////////////////////////////////////////////////////////

///// Imports ////////////////////////////////////

/// Local ///
#include "rl_toybox.hpp" // Also includes "utils.cpp"

///// Aliases ////////////////////////////////////
typedef array<float,2> eff_flo;


////////// BOND GRAPH VISUALIZATION ////////////////////////////////////////////////////////////////
const int _FONT_SIZE = 20; 
const int _NODE_SIZE = 30; 

class Render_BG{ public:
    // Base class for Bond Graph component rendering strategy

    /// Node Members ///

    Vector2 posn; // Position on the screen
    float   size; // Diameter
    string  symb; // Identifying symbol
    string  text; // Flavor text and/or label

    /// Edge Members ///

    Vector2 aBgn;
    Vector2 aEnd;
    Vector2 barb;

    /// Constructor(s) ///

    Render_BG( const Vector2& posn_, float size_, string symb_, string text_ = "" ){
        // Node painter base params
        posn = posn_;
        size = size_;
        symb = symb_;
        text = text_;
    }

    Render_BG( const Vector2& aBgn_, const Vector2& aEnd_, const Vector2& barb_ ){
        // Edge painter base params
        aBgn = aBgn_;
        aEnd = aEnd_;
        barb = barb_;
    }

    /// Methods ///

    virtual void draw_text(){
        // Draw the text associated with this node
        DrawText( symb.c_str(), 
                  (int) posn.x - MeasureText( symb.c_str(), _FONT_SIZE)/2, 
                  (int) posn.y - _FONT_SIZE/2, 
                  _FONT_SIZE, 
                  BLACK );
        DrawText( text.c_str(), 
                  (int) posn.x - MeasureText( text.c_str(), _FONT_SIZE)/2, 
                  (int) posn.y + _FONT_SIZE, 
                  _FONT_SIZE, 
                  BLACK );
    }

    virtual void draw(){  DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, RED );  }
};

class Draw0Junc : public Render_BG { public:
    /// Constructor(s) ///
    Draw0Junc( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "0", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, LIGHTGRAY );  
        draw_text();
    }
};

class Draw1Junc : public Render_BG { public:
    /// Constructor(s) ///
    Draw1Junc( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "1", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, LIGHTGRAY );  
        draw_text();
    }
};

class DrawXformer : public Render_BG { public:
    /// Constructor(s) ///
    DrawXformer( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "TF", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, BEIGE );  
        draw_text();
    }
};

class DrawInertia : public Render_BG { public:
    /// Constructor(s) ///
    DrawInertia( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "I", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, GREEN );  
        draw_text();
    }
};

class DrawCapacitor : public Render_BG { public:
    /// Constructor(s) ///
    DrawCapacitor( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "C", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, SKYBLUE );  
        draw_text();
    }
};

class DrawResistor : public Render_BG { public:
    /// Constructor(s) ///
    DrawResistor( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "R", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, MAGENTA );  
        draw_text();
    }
};

class DrawEffortSource : public Render_BG { public:
    /// Constructor(s) ///
    DrawEffortSource( const Vector2& posn_, float size_, string text_ = "" ) : Render_BG( posn_, size_, "Se", text_ ){}

    void draw(){  
        DrawCircle( (int) posn.x, (int) posn.y, size/2.0f, YELLOW );  
        draw_text();
    }
};


////////// BOND GRAPH COMPONENTS ///////////////////////////////////////////////////////////////////

///// Forward Declarations ///////////////////////

class  Node_BG;    typedef shared_ptr<Node_BG>   nodePtr;
struct Edge_BG;    typedef shared_ptr<Edge_BG>   edgePtr;
class  Render_BG;  typedef shared_ptr<Render_BG> rndrPtr;
class  Law_Node;   typedef shared_ptr<Law_Node>  lawNodPtr;

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
    vector<Flow_Dir> flowDirs; //- Direction of each flow (Not used?)
    ubyte /*------*/ portLim; // - Limit on number of ports, (0 = no limit)
    lawNodPtr /*--*/ consLaw; // - Constitutive Law
    rndrPtr /*----*/ painter; // - Rendering strategy 

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

    /// Methods ///

    void connect_edge( edgePtr edge, Flow_Dir dir ){
        ports.push_back( edge );
        flowDirs.push_back( dir );
    }

    void draw(){
        // If the rendering strat exists, Then run it!
        if( painter ){  painter->draw();  }
    }
};

class DrawEdgeBasic : public Render_BG { public:
    DrawEdgeBasic( const Vector2& bgn, const Vector2& end, const Vector2& brb ) : Render_BG( bgn, end, brb ){}

    void draw(){
        DrawLineEx( aBgn, aEnd, 4, BLACK );
        DrawLineEx( aEnd, barb, 4, BLACK );
    }
};

class Edge_BG{ public:
    // A pipe for effort and flow

    /// End 1 ///
    nodePtr  end1;
    Flow_Dir dir1;
    float    e1;
    float    f1;

    /// End 2 ///
    nodePtr  end2;
    Flow_Dir dir2;
    float    e2;
    float    f2;

    /// Common ///
    rndrPtr painter; // - Rendering strategy 

    /// Methods ///

    void draw(){
        // If the rendering strat exists, Then run it!
        if( painter ){  painter->draw();  }
    }
};

edgePtr make_edge( nodePtr src, nodePtr dst ){
    // Connect nodes with an edge and return a pointer to the edge
    Vector2 cntrSrc = src->painter->posn;
    Vector2 cntrDst = dst->painter->posn;
    Vector2 directn = Vector2Normalize( Vector2Subtract( cntrDst, cntrSrc ) );
    float   angle   = atan2f( directn.y, directn.x );
    float   rottn   = 3.0f*M_PI/4.0f;
    if( randf() < 0.5 )  rottn *= -1.0f;
    Vector2 crossDr = { cosf( angle + rottn ), sinf( angle + rottn ) };
    Vector2 bgnSorc = Vector2Add( cntrSrc, Vector2Scale( directn, _NODE_SIZE * 1.25 ) );
    Vector2 endDest = Vector2Subtract( cntrDst, Vector2Scale( directn, _NODE_SIZE * 1.25 ) );
    Vector2 barbDst = Vector2Add(  Vector2Subtract( endDest, Vector2Scale( directn, _NODE_SIZE*0.75 ) ),
                                   Vector2Scale( crossDr, _NODE_SIZE*0.75 )  );
    edgePtr rtnPtr = edgePtr( new Edge_BG{} );
    rtnPtr->end1 = src;
    src->connect_edge( rtnPtr, OUTPUT );
    rtnPtr->end2 = dst;
    dst->connect_edge( rtnPtr, INPUT );
    rtnPtr->painter = rndrPtr( new DrawEdgeBasic{ bgnSorc, endDest, barbDst } );
    return rtnPtr;
}

nodePtr make_1_junction( const Vector2& location, string name_ = "1-Junction" ){
    // Return a blank 1-Junction
    nodePtr rtnPtr = nodePtr( new Node_BG{} );
    rtnPtr->type    = JUNCTN_0;
    rtnPtr->name    = name_;
    rtnPtr->portLim = 0;
    rtnPtr->painter = rndrPtr( new Draw1Junc( location, _NODE_SIZE, name_ ) );
    return rtnPtr;
}

nodePtr make_transformer( const Vector2& location, string name_ = "Transformer" ){
    // Return a blank Transformer
    nodePtr rtnPtr = nodePtr( new Node_BG{} );
    rtnPtr->type    = XFORMER;
    rtnPtr->name    = name_;
    rtnPtr->portLim = 2;
    rtnPtr->painter = rndrPtr( new DrawXformer( location, _NODE_SIZE*1.25f, name_ ) );
    return rtnPtr;
}

nodePtr make_inertia( const Vector2& location, string name_ = "Inertia" ){
    // Return a blank 1-Junction
    nodePtr rtnPtr = nodePtr( new Node_BG{} );
    rtnPtr->type    = INERTIA;
    rtnPtr->name    = name_;
    rtnPtr->portLim = 1;
    rtnPtr->painter = rndrPtr( new DrawInertia( location, _NODE_SIZE, name_ ) );
    return rtnPtr;
}

nodePtr make_capacitor( const Vector2& location, string name_ = "Capacitor" ){
    // Return a blank 1-Junction
    nodePtr rtnPtr = nodePtr( new Node_BG{} );
    rtnPtr->type    = CAPACITR;
    rtnPtr->name    = name_;
    rtnPtr->portLim = 1;
    rtnPtr->painter = rndrPtr( new DrawCapacitor( location, _NODE_SIZE, name_ ) );
    return rtnPtr;
}

nodePtr make_resistor( const Vector2& location, string name_ = "Resistor" ){
    // Return a blank 1-Junction
    nodePtr rtnPtr = nodePtr( new Node_BG{} );
    rtnPtr->type    = RESISTOR;
    rtnPtr->name    = name_;
    rtnPtr->portLim = 1;
    rtnPtr->painter = rndrPtr( new DrawResistor( location, _NODE_SIZE, name_ ) );
    return rtnPtr;
}

nodePtr make_effort_source( const Vector2& location, string name_ = "Source_e" ){
    // Return a blank Transformer
    nodePtr rtnPtr = nodePtr( new Node_BG{} );
    rtnPtr->type    = SRC_EFRT;
    rtnPtr->name    = name_;
    rtnPtr->portLim = 1;
    rtnPtr->painter = rndrPtr( new DrawEffortSource( location, _NODE_SIZE*1.25f, name_ ) );
    return rtnPtr;
}

///// 1-Port Node Laws ///////////////////////////

eff_flo error_energy(){  return {nanf(""), nanf("")};  }

class Law_Node{ public:
    // Base class for all node Constitutive Laws
    virtual eff_flo operator()( nodePtr node ){  return error_energy();  }
}; 

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




////////// RACK AND PINION EXAMPLE /////////////////////////////////////////////////////////////////
// Section 3, Slide 13+

int main(){

    ///// Raylib Init /////////////////////////////////////////////////////

    /// RNG Init ///
    rand_seed();

    /// Window Init ///
    InitWindow( 900, 900, "Bond Graphs, Ver. 0.1" );
    SetTargetFPS( 60 );

    ///// Create Objects //////////////////////////////////////////////////

    Camera2D camera{};
    camera.target   = {0,0};
    camera.offset   = {450,450};
    camera.zoom     = 1.0;
    camera.rotation = 0.0;

    nodePtr /*---*/ source = make_effort_source( {-400,   0}, "tau_in(t)" );
    nodePtr /*---*/ omega1 = make_1_junction(    {-200,   0}, "omega_1"    );
    nodePtr /*---*/ veloc1 = make_1_junction(    { 200,   0}, "velocity_1" );
    nodePtr /*---*/ xformr = make_transformer(   {   0,   0}, "v_1 = r * om_1" );
    nodePtr /*---*/ inertJ = make_inertia(       {-200, 200}, "J" );
    nodePtr /*---*/ inertM = make_inertia(       { 400,   0}, "m" );
    nodePtr /*---*/ capctr = make_capacitor(     { 200, 200}, "1/k" );
    nodePtr /*---*/ resstr = make_resistor(      { 400, 200}, "b" );
    nodePtr /*---*/ resTau = make_resistor(      {-400, 200}, "b_tau" );
    vector<edgePtr> edges;
    edges.push_back( make_edge( source, omega1 ) );
    edges.push_back( make_edge( omega1, xformr ) );
    edges.push_back( make_edge( xformr, veloc1 ) );
    edges.push_back( make_edge( omega1, inertJ ) );
    edges.push_back( make_edge( omega1, resTau ) );
    edges.push_back( make_edge( veloc1, inertM ) );
    edges.push_back( make_edge( veloc1, capctr ) );
    edges.push_back( make_edge( veloc1, resstr ) );

    ///////// RENDER LOOP //////////////////////////////////////////////////////////////////////////

    while( !WindowShouldClose() ){

        /// Begin Drawing ///
        BeginDrawing();
        BeginMode2D( camera );
        ClearBackground( RAYWHITE );

        ///// DRAW LOOP ///////////////////////////////////////////////////

        omega1->draw();
        veloc1->draw();
        xformr->draw();
        inertJ->draw();
        inertM->draw();
        capctr->draw();
        resstr->draw();
        resTau->draw();
        source->draw();
        for( edgePtr& edge : edges ){  edge->draw();  }

        ///// END DRAWING /////////////////////////////////////////////////

        /// End Drawing ///
        EndMode2D();
        EndDrawing();
    }

    return 0;
}

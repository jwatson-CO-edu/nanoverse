package org.game;

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

import static java.util.UUID.randomUUID;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class GameObject {

    /// Member Vars ///
    protected String type; 
    protected String id; 
    protected Engine game; // Container
    protected int[]  address; // Address and/or screen coordinates
    public    char   rep; // How to draw this item in text
    
    
    /// Constructor(s) ///
    GameObject( Engine g, String typ ){  
        game = g;
        type = typ;
        id   = randomUUID().toString();
    }

    /// (Virtual) Methods ///
    int update(){  return 0;  };
}
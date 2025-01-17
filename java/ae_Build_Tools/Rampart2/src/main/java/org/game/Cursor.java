package org.game;

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

// import Helpers.Utils;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class Cursor extends GameObject {

    /// Member Vars ///
    protected int status; // What is this object doing or feeling?

    /// Constructor(s) ///
    Cursor( Engine g, int[] addr ){
        super( g, "cursor" );
        address = addr.clone();
    }

    /// Methods ///
    
    public boolean move_UP(){
        int[] nuAddr = address.clone();
        nuAddr[1] += 1;
        if( game.p_valid_addr( nuAddr ) ) // FIXME, START HERE: HANDLE ATTEMPT TO MOVE UP
    }
}

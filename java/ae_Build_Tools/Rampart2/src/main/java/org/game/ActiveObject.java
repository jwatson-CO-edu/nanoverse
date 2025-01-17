package org.game;

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

// import Helpers.Utils;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class ActiveObject extends GameObject {

    /// Member Vars ///
    protected float health; // Remaining robustness of this object
    protected int   status; // What is this object doing or feeling?
    protected int[] target; // Map-space intent of this object

    /// Constructor(s) ///
    ActiveObject( Engine g, String t ){
        super( g, t );
    }
}

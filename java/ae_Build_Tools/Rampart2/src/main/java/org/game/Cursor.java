///////// INIT /////////////////////////////////////////////////////////////////////////////////////

package org.game;



///////// WORLD UI FOCUS ///////////////////////////////////////////////////////////////////////////

public class Cursor extends GameObject {

    /// Member Vars ///
    protected int status; // What is this object doing or feeling?

    /// Constructor(s) ///
    Cursor( Engine g, int[] addr ){
        super( g, "cursor" );
        address = addr.clone();
    }

    /// Methods ///
    
    protected boolean test_move( int[] delta ){
        // Attempt to move the cursor by `delta`
        int[] nuAddr = address.clone();
        boolean res  = false;
        nuAddr[0] += delta[0];
        nuAddr[1] += delta[1];
        if( game.p_valid_addr( nuAddr ) ){
            res     = true;
            address = nuAddr.clone();
        }
        return res;
    }
    
    public boolean move_UP(){
        // Attempt to move the cursor UP
        int[] del = {0,1};
        return test_move( del );
    }

    public boolean move_DOWN(){
        // Attempt to move the cursor DOWN
        int[] del = {0,-1};
        return test_move( del );
    }

    public boolean move_RIGHT(){
        // Attempt to move the cursor RIGHT
        int[] del = {1,0};
        return test_move( del );
    }

    public boolean move_LEFT(){
        // Attempt to move the cursor LEFT
        int[] del = {-1,0};
        return test_move( del );
    }
}

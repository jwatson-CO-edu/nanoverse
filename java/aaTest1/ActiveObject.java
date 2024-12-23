/*
 * ActiveObject.java
 * James Watson, 2024-12
 * Game things that can do things (Walls, Cannons, Enemies)
 */

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

// import Helpers.Utils;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class ActiveObject extends GameObject {

    /// Member Vars ///
    protected float   health; // Remaining robustness of this object
    protected int     status; // What is this object doing or feeling?
    protected int[][] target; // Map-space intent of this object

    /// Constructor(s) ///
    ActiveObject( Engine g ){
        super(g);
    }
}

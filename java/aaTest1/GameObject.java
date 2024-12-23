/*
 * GameObject.java
 * James Watson, 2024-12
 * Superclass for something that can be somewhere
 */

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

// import java.util.ArrayList;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class GameObject {

    /// Member Vars ///
    protected String /*----------*/ type; 
    protected String /*----------*/ id; 
    protected Engine /*----------*/ game; // Container
    protected int[] /*-----------*/ address; // Address and/or screen coordinates
    // protected ArrayList<GameObject> neighbors; // Data structure for neighborhood calcs
    public    char /*------------*/ rep; // How to draw this item in text
    
    
    /// Constructor(s) ///
    GameObject( Engine g ){  game = g;  }

    /// (Virtual) Methods ///
    int update(){  return 0;  };
}
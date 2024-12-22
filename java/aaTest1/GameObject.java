/*
 * GameObject.java
 * James Watson, 2024-12
 * Superclass for something that can be somewhere
 */
package aaTest1;

import java.util.ArrayList;

public class GameObject {
    /// Member Vars ///
    protected String /*----------*/ type; 
    protected String /*----------*/ id; 
    protected Engine /*----------*/ game; // Container
    protected int[] /*-----------*/ address; // Address and/or screen coordinates
    protected ArrayList<GameObject> neighbors; // Data structure for neighborhood calcs
    

    GameObject( Engine g ){  game = g;  }
}
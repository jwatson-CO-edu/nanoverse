package aaTest1;

import java.util.HashMap;

public class Engine {

    /// Simulation ///
    private int /*-------------------*/ Wmap, Hmap;
    private HashMap<int[],ActiveObject> objects;
    private HashMap<int[],ActiveObject> bullets;
    private Tile[][] /*--------------*/ tileMap;

    Engine( int w, int h ){
        Wmap    = w;
        Hmap    = h;
        objects = new HashMap<int[],ActiveObject>();
        bullets = new HashMap<int[],ActiveObject>();
        tileMap = new Tile[w][h];
    }
}

/*
 * Engine.java
 * James Watson, 2024-12
 * Contains game logic
 */

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

import java.util.HashMap;
import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Map;

import static Helpers.Utils.clamp;
import static Helpers.Utils.coinflip_P;;


///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class Engine {

    /// Simulation ///
    private int /*-------------------*/ Wmap, Hmap;
    private HashMap<int[],ActiveObject> objects;
    private HashMap<int[],ActiveObject> bullets;
    private Tile[][] /*--------------*/ tileMap;
    static  float /*-----------------*/ P_init = 0.10f;
    static  float /*-----------------*/ P_nMax = 0.50f;
    static  float /*-----------------*/ P_nAdd = 0.50f;

    /// Constructor(s) ///
    Engine( int w, int h ){
        // Instantiate vars
        Wmap    = w;
        Hmap    = h;
        objects = new HashMap<int[],ActiveObject>();
        bullets = new HashMap<int[],ActiveObject>();
        tileMap = new Tile[w][h];
    }

    /// Methods ///
    
    ArrayList<int[]> get_neighborhood( int[] center ){
        // Return the 8 cell neighborhood of `center`, Respects the grid world boundaries
        ArrayList<int[]> rtnHood = new ArrayList<int[]>();
        int[] /*------*/ addr    = new int[2];
        int /*--------*/ ii, jj;
        for( int i = -1; i <= 1; ++i ){
            for( int j = -1; j <= 1; ++j ){
                ii = i + center[0];
                jj = j + center[1];
                if( ((i!=0) || (j!=0)) && (ii>=0) && (jj>=0) && (ii<Wmap) && (jj<Hmap) ){
                    addr[0] = ii;
                    addr[1] = jj;
                    rtnHood.add( addr.clone() );
                }
            }
        }
        return rtnHood;
    }
    
    public void gen_map( int[] landSeed, float bgnMargin, float decay ){
        // Generate the map, Procedurally
        ArrayDeque<int[]> frontier = new ArrayDeque<int[]>();
        ArrayList<int[]>  nghbrs;
        int[] /*-------*/ currAddr;
        int /*---------*/ ii, jj;
        int /*---------*/ gCount;
        int /*---------*/ Nstep = 0;

        /// Stage 1: Seed Random Tiles ///
        for( int i = 0; i < Wmap; ++i ){
            for( int j = 0; j < Hmap; ++j ){
                if( coinflip_P( P_init ) ){
                    tileMap[i][j] = Tile.make_grass( this );
                }else{
                    tileMap[i][j] = Tile.make_water( this );
                }
            }
        }

        /// Stage 2: Grow Clumps
        frontier.add( landSeed );
        while( frontier.size() > 0 ){

            // 1. Pop from frontier
            currAddr = frontier.pop();
            ii /*-*/ = currAddr[0];
            jj /*-*/ = currAddr[1];
            tileMap[ii][jj].visited = true;

            // 2. Calc grass probability from neighborhood
            nghbrs = get_neighborhood( currAddr );
            gCount = 0;
            for( int[] a : nghbrs ){
                if( tileMap[ a[0] ][ a[1] ].type == "grass" ){
                    ++gCount;
                }
            }
        }
    }

    public void add_object( int[] addr, ActiveObject obj, boolean isBullet ){
        if( !isBullet ){
            obj.address = addr;
            objects.put( addr, obj );
        }else{
            obj.address = addr;
            bullets.put( addr, obj );
        }
    }


    public void update(){
        // https://stackoverflow.com/a/1066607
        for (Map.Entry<int[],ActiveObject> entry : bullets.entrySet()) {
            int[] /*-*/  key   = entry.getKey();
            ActiveObject value = entry.getValue();
            // ...
        }
    }
}

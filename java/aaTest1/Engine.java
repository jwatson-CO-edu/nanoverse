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
    static  float /*-----------------*/ P_init = 0.05f;

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
        // Return the 8 cell neighborhood of `center`
        ArrayList<int[]> rtnHood = new ArrayList<int[]>();
        int[] addr = new int[2];
        for( int i = -1; i <= 1; ++i ){
            for( int j = -1; j <= 1; ++j ){
                if( (i!=0) || (j!=0) ){
                    addr[0] = clamp( i + center[0], 0, Wmap-1 );
                    addr[1] = clamp( j + center[1], 0, Hmap-1 );
                    rtnHood.add( addr.clone() );
                }
            }
        }
        return rtnHood;
    }
    
    public void gen_map( int[] landSeed, float bgnMargin, float decay ){
        // Generate the map
        ArrayDeque<int[]> frontier = new ArrayDeque<int[]>();
        int[] currAddr;

        /// Stage 1: Seed Random Tiles ///
        for( int i = 0; i < Wmap; ++i ){
            for( int j = 0; j < Hmap; ++j ){
                if( coinflip_P( 0.05f ) ){

                }
            }
        }

        /// Stage 2: Grow Clumps
        frontier.add( landSeed );
        while( frontier.size() > 0 ){
            currAddr = frontier.pop();
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

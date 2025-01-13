

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

import java.util.HashMap;
import java.util.ArrayList;
import java.util.ArrayDeque;
// import java.util.Map;

import static Helpers.Utils.coinflip_P;;


///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class Engine {

    /// Simulation ///
    protected int /*-------------------*/ Wmap, Hmap;
    private   HashMap<int[],ActiveObject> objects;
    private   HashMap<int[],ActiveObject> bullets;
    protected Tile[][] /*--------------*/ tileMap;
    static    float /*-----------------*/ P_nMax = 0.50f; // Max coinflip probability for grassy neighborhood
    static    int /*-------------------*/ gThresh = 5;
    static    int /*-------------------*/ wThresh = 6;

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
    
    public void gen_map( int[] landSeed, float bgnProb, float bgnMargin, float decay ){
        // Generate the map, Procedurally
        ArrayDeque<int[]> frontier = new ArrayDeque<int[]>();
        ArrayList<int[]>  nghbrs;
        int[] /*-------*/ currAddr;
        int /*---------*/ ii, jj;
        int /*---------*/ gCount;
        int /*---------*/ wCount;
        float /*-------*/ P_step = bgnProb;

        /// Stage 1: Seed Random Tiles ///
        for( int i = 0; i < Wmap; ++i ){
            for( int j = 0; j < Hmap; ++j ){
                if( coinflip_P( bgnProb ) ){
                    tileMap[i][j] = Tile.make_Tile( this, "grass" );  
                }else{
                    tileMap[i][j] = Tile.make_Tile( this, "water" );  
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
            // tileMap[ii][jj].visited = true;

            // 2. Calc grass probability from neighborhood
            nghbrs = get_neighborhood( currAddr );
            gCount = 0;
            for( int[] a : nghbrs ){
                if( !tileMap[ a[0] ][ a[1] ].p_visited() ){  frontier.add( a.clone() );  }
                if( tileMap[ a[0] ][ a[1] ].type == "grass" ){
                    ++gCount;
                }
            }
            if( coinflip_P( ((float) gCount / 8.0f) * P_nMax + P_step ) ){
                tileMap[ii][jj] = Tile.make_Tile( this, "grass" );  
            }else{
                tileMap[ii][jj] = Tile.make_Tile( this, "water" );  
            }
            tileMap[ii][jj].visit();
            P_step *= (1.0f - decay);
        }

        /// Stage 3: Fill in Gaps ///
        for( int i = 0; i < Wmap; ++i ){
            for( int j = 0; j < Hmap; ++j ){
                currAddr = new int[]{ i, j };
                nghbrs   = get_neighborhood( currAddr );
                gCount   = 0;
                wCount   = 0;
                for( int[] a : nghbrs ){
                    if( tileMap[ a[0] ][ a[1] ].type == "grass" ){  ++gCount;  }
                    else if( tileMap[ a[0] ][ a[1] ].type == "water" ){  ++wCount;  }
                }
                if( gCount >= gThresh ){  
                    tileMap[i][j] = Tile.make_Tile( this, "grass" );  
                    tileMap[i][j].visit();
                }
                if( wCount >= wThresh ){  
                    tileMap[i][j] = Tile.make_Tile( this, "water" );  
                    tileMap[i][j].visit();
                }
            }
        }
    }

    public void print(){
        // Output the current map state to the terminal
        for( int j = 0 ; j < Wmap ; j++ ){
            for( int i = 0 ; i < Hmap ; i++ ){
                System.out.print( tileMap[j][i].rep ); System.out.print( ' ' );
            }
            System.out.println();
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


    // public void update(){
    //     // https://stackoverflow.com/a/1066607
    //     for (Map.Entry<int[],ActiveObject> entry : bullets.entrySet()) {
    //         int[] /*-*/  key   = entry.getKey();
    //         ActiveObject value = entry.getValue();
    //         // ...
    //     }
    // }
}

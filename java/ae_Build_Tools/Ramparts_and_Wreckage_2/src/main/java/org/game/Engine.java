///////// INIT /////////////////////////////////////////////////////////////////////////////////////

package org.game;

import java.util.HashMap;
import java.util.ArrayList;
import java.util.ArrayDeque;
// import java.util.Map;

import static Helpers.Utils.coinflip_P;


///////// STATES && COMMANDS ///////////////////////////////////////////////////////////////////////

enum GameState {
    // Governs game behavior thru different interaction phases
    BUILD,
}

enum GameCmd {
    // Valid commands the engine receives (and sends)

    /// General Purpose ///
    NO_OP,
    ERROR,

    /// `BUILD` Phase ///
    PLACEMENT_CONFIRM,
    PLACEMENT_NEXT,
    CURSOR_NORTH, 
    CURSOR_EAST, 
    CURSOR_SOUTH, 
    CURSOR_WEST,
}



///////// MVC MODEL: GAME LOGIC / ENGINE ///////////////////////////////////////////////////////////


// public class Engine implements MessageSystem, Observer {
public class Engine {

    /// Class Vars ///
    static float /*-----------------*/ P_nMax    = 0.50f; // Max coinflip probability for grassy neighborhood
    static int /*-------------------*/ gThresh   = 5;
    static int /*-------------------*/ wThresh   = 6;
    static Dir[] /*-----------------*/ cardinals = { Dir.NORTH, Dir.EAST, Dir.SOUTH, Dir.WEST, };

    /// Simulation ///
    protected int /*-------------------*/ Wmap, Hmap;
    protected HashMap<int[],ArrayList<ActiveObject>> objects;
//    private   HashMap<int[],ActiveObject> bullets;
    protected Tile[][] /*--------------*/ tileMap;
    
    // private   ArrayList<String> /*-----*/ eventLog;
    /// Interaction ///
    protected Cursor    cursor;
    protected Blueprint blcBP;
    protected GameState state;
    public    Message   cmd;
    /// Messages ///
    // protected ArrayDeque<Message> /*-----------*/ msgQ;
    // protected HashMap<String,ArrayList<Observer>> observers;

    /// Constructor(s) ///
    Engine( int w, int h ){
        int[] bgnAdr = { (int)w/2, (int)h/2 };
        // Instantiate vars
        Wmap    = w;
        Hmap    = h;
        objects = new HashMap<int[],ArrayList<ActiveObject>>();
//        bullets = new HashMap<int[],ActiveObject>();
        tileMap = new Tile[w][h];
        // Instantiate UI
        cursor = new Cursor( this, bgnAdr );
        blcBP  = null;
        state  = GameState.BUILD;
        next_template();
        // Start `MessageSystem`
        // msgQ /**/ = new ArrayDeque<Message>();
        // observers = new HashMap<String,ArrayList<Observer>>();
    }


    public BlcPn next_template(){
        // The user does not get to choose a template
        blcBP = Blueprint.random_template( this );
        return blcBP.shape;
    }


    public void make_object( int[] addr, String type ){
        // Make a single `Wall` segment
        // Fetch, Create new vector if null
        ArrayList<ActiveObject> objLst = objects.get( addr );
        if( objLst == null ){  
            objLst = new ArrayList<ActiveObject>(); 
            objects.put( addr.clone(), objLst );
        }
        // Add the appropriate object at the addressed array
        switch( type ){
            case "wall":
                objLst.add( new Wall( this, addr.clone() ) );        
                break;
            default:
                System.out.println( String.format( "CANNOT add grid object of type %s", type ) );
                break;
        }    
    }


    public String get_terrain_type( int[] addr ){
        // Get the `Tile` type for a valid address, Otherwise return "INVALID"
        String rtnStr = "INVALID";
        if( p_valid_addr( addr ) ){
            rtnStr = tileMap[ addr[0] ][ addr[1] ].type;
        }
        return rtnStr;
    }


    public int emplace_template(){
        // Create `Wall` segments, Report how many `Wall` segments were created on the map!
        int consumed = 0;
        if( blcBP != null ){
            for( int[] spot : blcBP.actual ){
                if( get_terrain_type( spot ) == "grass" ){ // `get_terrain_type` performs `p_valid_addr( spot )`
                    make_object( spot, "wall" );
                    consumed++;
                }
            }
        }
        return consumed;
    }


    /// Methods ///
    
    public boolean p_valid_addr( int ii, int jj ){
        // Is <ii,jj> a valid address?
        return ((ii>=0) && (jj>=0) && (ii<Wmap) && (jj<Hmap));
    }


    public boolean p_valid_addr( int[] addr ){
        // Is {ii,jj} a valid address?
        int ii = addr[0];
        int jj = addr[1];
        return p_valid_addr( ii, jj );
    }

    
    ArrayList<int[]> get_neighborhood( int[] center ){
        // Return the 8 cell neighborhood of `center`, Respects the grid world boundaries
        ArrayList<int[]> rtnHood = new ArrayList<int[]>();
        int[] /*------*/ addr    = new int[2];
        int /*--------*/ ii, jj;
        for( int i = -1; i <= 1; ++i ){
            for( int j = -1; j <= 1; ++j ){
                ii = i + center[0];
                jj = j + center[1];
                if( ((i!=0) || (j!=0)) && p_valid_addr( ii, jj ) ){
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
        while( !frontier.isEmpty() ){

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


    /// MessageSystem ///
    
    

//    public void add_object( int[] addr, ActiveObject obj, boolean isBullet ){
//        if( !isBullet ){
//            obj.address = addr;
//            objects.put( addr, obj );
//        }else{
//            obj.address = addr;
//            bullets.put( addr, obj );
//        }
//    }


    public boolean handle_command( Message input ){

        /// General Commands ///
        if( input.topic == "kb" ){
            switch( input.cmd ){

                case CURSOR_NORTH:
                    cursor.move_UP();
                    break;

                case CURSOR_SOUTH:
                    cursor.move_DOWN();
                    break;

                case CURSOR_WEST:
                    cursor.move_LEFT();
                    break;
            
                case CURSOR_EAST:
                    cursor.move_RIGHT();
                    break;

                default:
                    break;
            }
        }

        /// State-Specific Commands ///
        switch( state ){
            case BUILD:{
                    switch ( input.cmd ) {

                        case PLACEMENT_NEXT:
                            next_template();
                            break;
                    
                        case PLACEMENT_CONFIRM:
                            next_template();
                            break;

                        default:
                            break;
                    }
                }break;
        
            default:
                break;
        }
        return true;
    }


    public GameState update( Message input ){
        // Run one step of the engine state machine
        GameState nxtState = GameState.BUILD;

        /// Handle the command ///
        handle_command( input );

        /// State Machine ///
        switch( state ){
            case BUILD:
                blcBP.transform_template( cursor.address );
                nxtState = GameState.BUILD;
                break;
        
            default:
                break;
        }

        return nxtState;
    }
}

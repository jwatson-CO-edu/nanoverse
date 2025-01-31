

///////// INIT /////////////////////////////////////////////////////////////////////////////////////

package org.game;

import java.awt.Color;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class Tile extends GameObject {

    /// Members ///
    private   boolean visited;
    protected Color   clr;
    
    /// Constructor(s) ///
    Tile( Engine g, String t, char r, Color c ){
        super( g, t );
        rep     = r;
        clr     = c;
        visited = false;
    }

    /// Factory Function ///
    
    static Tile make_Tile( Engine g, String tileType ){
        // Return a tile of the specified type
        Tile rtnTile;
        switch(tileType) {
            case "grass":
                rtnTile = new Tile( g, "grass", '"', Color.GREEN );        
                break;
                
            case "water":
                rtnTile = new Tile( g, "water", '~', Color.BLUE );
                break;

            default:
                rtnTile = new Tile( g, "INVALID", '"', Color.BLACK );        
        }
        return rtnTile;
    }

    /// Methods ///
    public void    visit(){  visited = true;  }
    public void    unvisit(){  visited = false;  }
    public boolean p_visited(){  return visited;  }
}

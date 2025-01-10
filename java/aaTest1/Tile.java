

///////// INIT /////////////////////////////////////////////////////////////////////////////////////
import java.awt.Color;



///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class Tile extends GameObject {

    /// Members ///
    boolean visited;
    Color   clr;
    
    /// Constructor(s) ///
    Tile( Engine g, String t, char r, Color c ){
        super( g, t );
        rep     = r;
        clr     = c;
        visited = false;
    }

    static Tile make_grass( Engine g ){
        // Return a `Tile` representing grass
        return new Tile( g, "grass", '"', Color.GREEN );
    }

    static Tile make_water( Engine g ){
        // Return a `Tile` representing grass
        return new Tile( g, "water", '~', Color.BLUE );
    }
}

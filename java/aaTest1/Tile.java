/*
 * Tile.java
 * James Watson, 2024-12
 * Represents terrain or water
 */

///////// INIT /////////////////////////////////////////////////////////////////////////////////////




///////// CLASS DEF ////////////////////////////////////////////////////////////////////////////////

public class Tile extends GameObject {

    /// Members ///
    boolean visited;
    
    /// Constructor(s) ///
    Tile( Engine g, String t, char r ){
        super( g, t );
        rep     = r;
        visited = false;
    }

    static Tile make_grass( Engine g ){
        // Return a `Tile` representing grass
        return new Tile( g, "grass", '"' );
    }

    static Tile make_water( Engine g ){
        // Return a `Tile` representing grass
        return new Tile( g, "water", '~' );
    }
}

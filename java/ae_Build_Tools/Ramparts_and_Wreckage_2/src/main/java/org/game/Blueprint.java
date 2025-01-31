////////// INIT ////////////////////////////////////////////////////////////////////////////////////

package org.game;

import java.util.ArrayList;
import java.util.Random;



////////// ENUMS ///////////////////////////////////////////////////////////////////////////////////

enum Dir {
    NORTH,
    SOUTH,
    EAST,
    WEST,
}

enum BlcPn {
    I, O, T, S, Z, J, L, 
    SINGLE,
}



////////// PRESUMPTIVE WALL PLAN ///////////////////////////////////////////////////////////////////

public class Blueprint extends ActiveObject {

    protected ArrayList<int[]> template;
    protected ArrayList<int[]> actual;
    protected Dir /*--------*/ drctn;
    protected BlcPn /*------*/ shape;
    private static Random random = new Random();
    

    public Blueprint( Engine g, BlcPn t ){
        super( g, "Blueprint" );
        shape = t;
        drctn = Dir.NORTH;
        template = new ArrayList<int[]>();
        template.add( new int[] {0,0} );
        switch( t ){
            // Tetrominoes
            case I:
                template.add( new int[] {0,1} );
                template.add( new int[] {0,2} );
                template.add( new int[] {0,3} );
                break;
            case O:
                template.add( new int[] {1,0} );
                template.add( new int[] {0,1} );
                template.add( new int[] {1,1} );
                break;
            case T:
                template.add( new int[] { 0,1} );
                template.add( new int[] {-1,0} );
                template.add( new int[] { 1,0} );
                break;
            case S:
                template.add( new int[] { 0,1} );
                template.add( new int[] { 1,1} );
                template.add( new int[] {-1,0} );
                break;
            case Z:
                template.add( new int[] { 0,1} );
                template.add( new int[] {-1,1} );
                template.add( new int[] { 1,0} );
                break;
            case J:
                template.add( new int[] { 0,1} );
                template.add( new int[] { 0,2} );
                template.add( new int[] {-1,0} );
                break;
            case L:
                template.add( new int[] {0,1} );
                template.add( new int[] {0,2} );
                template.add( new int[] {1,0} );
                break;
            // Smaller Pieces
            case SINGLE:
                // There is only one segment at the center
                break;
            
            default:
                System.out.println( String.format( "NULL BLOCK TEMPLATE!" ) );
                break;
        }
    }


    public static Blueprint random_template( Engine g ){
        // Return a random `Blueprint` with a uniform distribution
        BlcPn[] values = BlcPn.values();
        int     index  = random.nextInt( values.length );
        return new Blueprint( g, values[ index ] );
    }


    public void set_direction( Dir drct ){  drctn = drct;  }


    public void transform_template( int[] cntr ){
        // Calc block placements for template + cursor + rotation        
        actual = new ArrayList<int[]>(); // Erase last placement
        int[] currAddr = new int[] {0,0};
        switch( drctn ){
            case NORTH: // No rotation && Translation
                for( int[] offset : template ){
                    currAddr = offset.clone();
                    currAddr[0] += cntr[0];
                    currAddr[1] += cntr[1];
                    actual.add( currAddr.clone() );
                }
                break;
            case SOUTH: // Double flip && Translation
                for( int[] offset : template ){
                    currAddr[0] = cntr[0] - offset[0];
                    currAddr[1] = cntr[1] - offset[1];
                    actual.add( currAddr.clone() );
                }
                break;
            case EAST: // Transpose && Translation
                for( int[] offset : template ){
                    currAddr[0] = cntr[0] + offset[1];
                    currAddr[1] = cntr[1] + offset[0];
                    actual.add( currAddr.clone() );
                }
                break;
            case WEST: // Negative Transpose && Translation
                for( int[] offset : template ){
                    currAddr[0] = cntr[0] - offset[1];
                    currAddr[1] = cntr[1] - offset[0];
                    actual.add( currAddr.clone() );
                }
                break;
            default:
                System.out.println( String.format( "UNRECOGNIZED TEMPLATE TRANSFORM!" ) );
                break;
        }
    }
    
}

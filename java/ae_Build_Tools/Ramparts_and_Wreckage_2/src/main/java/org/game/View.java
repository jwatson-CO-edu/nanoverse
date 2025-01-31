////////// INIT ////////////////////////////////////////////////////////////////////////////////////

package org.game;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Map;



////////// GAME DISPLAY ////////////////////////////////////////////////////////////////////////////

public class View extends JPanel implements ActionListener {

    /// Members ///
    protected Engine game; // Simulation
    private   int    unitPx;
    private   int    Wwin, Hwin;
    private   Timer  timer;


    /// Getters ///
    public int get_width(){   
        // Width of the map in pixels
        System.out.println( String.format("Map Width: %d [px]" , Wwin ));
        return Wwin;  
    }


    public int get_height(){    
        // Height of the map in pixels
        System.out.println( String.format("Map Height: %d [px]" , Hwin ));
        return Hwin;  
    }


    /// Init Helpers ///
    public void disable_metal_styles(){
        // I don't actually know why the example does this! Maybe the dev just doesn't like it?
        try {
            UIManager.setLookAndFeel("javax.swing.plaf.metal.MetalLookAndFeel");
        } catch ( UnsupportedLookAndFeelException ex) {
            ex.printStackTrace();
        } catch ( IllegalAccessException ex ) {
            ex.printStackTrace();
        } catch ( InstantiationException ex ) {
            ex.printStackTrace();
        } catch ( ClassNotFoundException ex ) {
            ex.printStackTrace();
        }
        /* Turn off metal's use of bold fonts */
        UIManager.put("swing.boldMetal", Boolean.FALSE);  
    }


    /// Constructor(s) ///
    public View( Engine g, int gridUnit_px ) {
        // Connect the renderer to the game engine
        game   = g;
        unitPx = gridUnit_px;
        Wwin   = game.Wmap * unitPx;
        Hwin   = game.Hmap * unitPx;
        // Set up a timer to trigger actionPerformed periodically
        timer = new Timer( 24, this );
        timer.start();
        // Set up UI style
        disable_metal_styles();
    }


    public int[] get_address_upper_left_px( int i, int j ){
        // DRY: This is used by every draw method
        int[] r = {i*unitPx, Hwin-(j+1)*unitPx};
        return r;
    }


    public int[] get_address_upper_left_px( int[] addr ){
        // DRY: This is used by every draw method
        int i = addr[0];
        int j = addr[1];
        return get_address_upper_left_px( i, j );
    }


    /// Display Methods ///
    public void paint_terrain( Graphics2D g2d ){
        // Draw the Game Map
        int[] aPx;
        int   xPx, yPx;
        for( int j = 0 ; j < game.Wmap ; j++ ){
            for( int i = 0 ; i < game.Hmap ; i++ ){
                aPx = get_address_upper_left_px( j, i );
                xPx = aPx[0];
                yPx = aPx[1];
                g2d.setColor( game.tileMap[j][i].clr );
                g2d.fillRect( xPx, yPx, unitPx, unitPx );
            }
        }
    }


    public void paint_grid_objects( Graphics2D g2d ){
        // Draw all grid-bound objects
        
        // Temp vars
        int[] /*-------------*/ aPx;
        int /*---------------*/ xPx, yPx;
        int /*---------------*/ i, j;
        int[] /*-------------*/ k;
        ArrayList<ActiveObject> v;

        // Define a draw order for object types
        ArrayList<String> /*-*/ objOrder = new ArrayList<String>();
        objOrder.add( "wall" );
        
        // For every address, draw the objects in order
        for( Map.Entry<int[],ArrayList<ActiveObject>> entry : game.objects.entrySet() ){
            k = entry.getKey();
            v = entry.getValue();
            i = k[0];
            j = k[1];
            // For every entry in the draw order, Iterate over the objects at the current address
            for( String type_j : objOrder ){
                // For every object at the current address, draw only the objects at this place in the draw order
                for( ActiveObject obj_k : v ){
                    if( obj_k.type == type_j ){
                        aPx = get_address_upper_left_px( i, j );
                        xPx = aPx[0];
                        yPx = aPx[1];
                        switch( type_j ){
                            case "wall":
                                g2d.setColor( Color.GRAY );
                                g2d.fillRect( xPx, yPx, unitPx, unitPx );
                                break;
                            default:
                                System.out.println( String.format( "NO draw routine for %s!", type_j ) );
                                break;
                        }
                    }
                }   
            }
        }
    }


    public void paint_cursor( Graphics2D g2d ){
        // Draw the Cursor
        int[] aPx = get_address_upper_left_px( game.cursor.address );
        int   xPx = aPx[0];
        int   yPx = aPx[1];
        g2d.setColor( Color.WHITE );
        g2d.setStroke( new BasicStroke(3) );
        g2d.drawRect( xPx, yPx, unitPx, unitPx );
    }


    public void paint_template( Graphics2D g2d ){
        // Draw the template at its current absolute pose
        int[] aPx;
        int   xPx, yPx;
        for( int[] absAddr : game.blcBP.actual ){
            aPx = get_address_upper_left_px( absAddr );
            xPx = aPx[0];
            yPx = aPx[1];
            g2d.setColor( Color.GRAY );
            g2d.fillOval( xPx, yPx, unitPx, unitPx );
        }
    }


    @Override
    protected void paintComponent( Graphics g ) {
        // Actually repaints the frame
        super.paintComponent(g);

        // Cast Graphics to Graphics2D for advanced features
        Graphics2D g2d = (Graphics2D) g;

        /// Draw Static Elements ///
        g2d.setColor( Color.BLACK );
        g2d.fillRect( 0, 0, Wwin, Hwin );
        paint_terrain( g2d );

        /// Draw Cursor ///
        paint_cursor( g2d );

        /// Draw Mode-Dependent Elements ///
        switch( game.state ){
            case BUILD:
                paint_template( g2d );
                break;
        
            default:
                System.out.println( String.format( "Drew a frame OUTSIDE of a game mode!" ) );
                break;
        }
    }


    @Override
    public void actionPerformed( ActionEvent e ) {
        // Repaint the panel to show the updated position
        repaint();
    }

}

package org.game;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;



public class View extends JPanel implements ActionListener  {

    /// Members ///
    protected Engine game; // Simulation
    private   int    unitPx;
    private   int    Wwin, Hwin;
    private   Timer  timer;


    /// Getters ///
    public int get_width(){   
        System.out.println( String.format("Map Width: %d [px]" , Wwin ));
        return Wwin;  
    }


    public int get_height(){    
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


    /// Display Methods ///
    public void paint_terrain( Graphics2D g2d ){
        // Draw the Game Map
        for( int j = 0 ; j < game.Wmap ; j++ ){
            for( int i = 0 ; i < game.Hmap ; i++ ){
                g2d.setColor( game.tileMap[j][i].clr );
                g2d.fillRect( j*unitPx, Hwin-(i+1)*unitPx, unitPx, unitPx );
            }
        }
    }


    public void paint_cursor( Graphics2D g2d ){
        // Draw the Cursor
        g2d.setColor( Color.WHITE );
        g2d.setStroke( new BasicStroke(3) );
        g2d.drawRect( game.cursor.address[0]*unitPx, Hwin-(game.cursor.address[1]+1)*unitPx, unitPx, unitPx );
    }


    @Override
    protected void paintComponent( Graphics g ) {
        super.paintComponent(g);

        // Cast Graphics to Graphics2D for advanced features
        Graphics2D g2d = (Graphics2D) g;

        // Set the background color to black
        g2d.setColor( Color.BLACK );
        g2d.fillRect( 0, 0, Wwin, Hwin );
        paint_terrain( g2d );
        paint_cursor( g2d );
    }


    @Override
    public void actionPerformed( ActionEvent e ) {
        // Repaint the panel to show the updated position
        repaint();
    }


    

}

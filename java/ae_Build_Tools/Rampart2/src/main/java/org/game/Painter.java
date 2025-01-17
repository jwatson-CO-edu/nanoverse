package org.game;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;





public class Painter extends JPanel implements KeyListener, ActionListener  {

    /// Members ///
    protected Engine game; // Container
    private   int    unitPx;
    private   int    Wwin, Hwin;
    private   Timer  timer;


    /// Getters ///
    public int get_width(){   return Wwin;  }
    public int get_height(){  return Hwin;  }


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
    public Painter( Engine g, int gridUnit_px ) {
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
        for( int j = 0 ; j < game.Wmap ; j++ ){
            for( int i = 0 ; i < game.Hmap ; i++ ){
                g2d.setColor( game.tileMap[j][i].clr );
                g2d.fillRect( j*unitPx, Hwin-(i+1)*unitPx, unitPx, unitPx    );
            }
        }
    }


    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Cast Graphics to Graphics2D for advanced features
        Graphics2D g2d = (Graphics2D) g;

        // Set the background color to black
        g2d.setColor( Color.BLACK );
        g2d.fillRect( 0, 0, Wwin, Hwin );
        paint_terrain( g2d );
    }


    @Override
    public void actionPerformed( ActionEvent e ) {
        // Repaint the panel to show the updated position
        repaint();
    }


    /// Interaction Methods ///

    /** Handle the key typed event from the text field. */
    public void keyTyped( KeyEvent e ) {
        // NO-OP, Req'd by interface
    }
    
    /** Handle the key pressed event from the text field. */
    public void keyPressed( KeyEvent e ) {
        // NO-OP, Req'd by interface
        int keyCode = e.getKeyCode();
        switch ( keyCode ) {

            case 38: // UP
                game.cursor.move_UP();
                break;

            case 40: // DOWN
                game.cursor.move_DOWN();
                break;

            case 37: // LEFT
                game.cursor.move_LEFT();
                break;

            case 39: // RIGHT
                game.cursor.move_RIGHT();
                break;
        
            default: // NO-OP
                break;
        }
    }
    
    /** Handle the key released event from the text field. */
    public void keyReleased( KeyEvent e ) {
        // NO-OP, Req'd by interface
    }

}

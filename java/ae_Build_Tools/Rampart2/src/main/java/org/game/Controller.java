package org.game;

import javax.swing.*;



import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;


// import java.awt.event.ActionEvent;
// import java.awt.event.ActionListener;



public class Controller implements KeyListener {

    /// Members ///
    protected Engine    game; // - Simulation
    // protected View /**/ view; // - Graphics Panel
    // protected JTextArea status; // Text Feedback


    /// Constructor(s) ///
    public Controller( Engine g ) {
        // Connect the UI to the game engine
        game = g;
    }

    

    /// Interaction Methods ///

    /** Handle the key typed event from the text field. */
    public void keyTyped( KeyEvent e ) {
        // NO-OP, Req'd by interface
    }
    
    /** Handle the key pressed event from the text field. */
    public void keyPressed( KeyEvent e ) {

        System.out.println( String.format("KEY PRESS" ) );

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
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

package org.game;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;


// import java.awt.event.ActionEvent;
// import java.awt.event.ActionListener;



////////// USER INPUT //////////////////////////////////////////////////////////////////////////////

public class Controller implements KeyListener {

    /// Members ///
    protected Engine  game; // - Simulation
    protected boolean fresh;


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

        fresh = true;

        int keyCode = e.getKeyCode();
        switch ( keyCode ) {

            case 38: // UP
                game.cmd = new Message("kb", GameCmd.CURSOR_NORTH, "up" );
                // game.
                break;

            case 40: // DOWN
                game.cmd = new Message("kb", GameCmd.CURSOR_SOUTH, "down" );
                // game.
                break;

            case 37: // LEFT
                game.cmd = new Message("kb", GameCmd.CURSOR_WEST, "left" );
                // game.
                break;

            case 39: // RIGHT
                game.cmd = new Message("kb", GameCmd.CURSOR_EAST, "right" );
                // game.
                break;
        
            default: // NO-OP
                break;
        }
    }
    
    /** Handle the key released event from the text field. */
    public void keyReleased( KeyEvent e ) {
        // NO-OP, Req'd by interface
    }

    public void update(){
        if( fresh ){  fresh = false;  }
        else{  game.cmd = new Message("general", GameCmd.NO_OP, "NO OP" );  }
    }

}
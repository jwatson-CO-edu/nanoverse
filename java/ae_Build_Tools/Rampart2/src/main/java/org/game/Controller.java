package org.game;

import javax.swing.*;

import java.awt.Dimension;
import java.awt.BorderLayout;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;


// import java.awt.event.ActionEvent;
// import java.awt.event.ActionListener;



public class Controller extends JFrame implements KeyListener {

    /// Members ///
    protected Engine    game; // - Simulation
    // protected View /**/ view; // - Graphics Panel
    // protected JTextArea status; // Text Feedback


    public static void createAndShowGUI( Engine g, View v ) {
        // Create and set up the window.
        Controller frame = new Controller( g );
        frame.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );
        
        // Set up the content pane.
        v.setSize( v.get_width(), v.get_height() );
        JTextArea status = new JTextArea();
        status.setEditable( false );
        JScrollPane scrollPane = new JScrollPane( status );
        scrollPane.setPreferredSize( new Dimension( 375, 10 ) );
        
        frame.add( v, BorderLayout.PAGE_START );
        frame.add( status, BorderLayout.CENTER );
        frame.add( scrollPane, BorderLayout.PAGE_END);
        
        // Arrange UI elements
        frame.pack();
        // Center the frame on screen
        frame.setLocationRelativeTo(null);
        // Display the window.
        frame.setVisible( true );
    }


    /// Constructor(s) ///
    public Controller( Engine g ) {
        super( "Ramparts and Wreckage 2" );
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
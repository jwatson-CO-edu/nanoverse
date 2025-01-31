///////// INIT /////////////////////////////////////////////////////////////////////////////////////

package org.game;

import javax.swing.*;

import java.awt.Dimension;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;


///////// MAIN /////////////////////////////////////////////////////////////////////////////////////


public class Rampart2 {

    private static void createAndShowGUI(){
        /// 1. Start Game Engine ///
        Engine  engine  = new Engine( 25, 25 ); // Game State and Events
        int[]   bgnTil  = {12,12};
        engine.gen_map( bgnTil, 0.85f, 0.50f, 0.001f);

        /// 1. Start Interface ///
        Controller ctrl = new Controller( engine );

        /// 2. Start Graphics ///
        View painter = new View( engine, 25 );

        /// 2. Start User Interface ///
        // Create and set up the window.
        JFrame frame = new JFrame( "Ramparts and Wreckage 2" );
        frame.addKeyListener( ctrl );
        frame.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );
        frame.setLayout( new GridBagLayout() );
        GridBagConstraints c = new GridBagConstraints();
        frame.setFocusable( true ); // https://coderanch.com/t/426782/java/keyPressed-working
        
        // Set up the content pane.
        painter.setPreferredSize( new Dimension( painter.get_width(), painter.get_height() ) );
        JTextArea status = new JTextArea();
        status.setEditable( false );
        status.setPreferredSize( new Dimension( (int)(painter.get_width()*0.9), (int)(painter.get_height()*0.25) ) );
        JScrollPane scrollPane = new JScrollPane( status );
        
        // Layout Graphics Pane
        c.fill /**/ = GridBagConstraints.BOTH;
        c.gridwidth  = 2;
        c.gridheight = 1;
        c.gridx     = 0;
        c.gridy     = 0;
        c.weightx   = 1.0f;
        c.weighty   = 1.0f;
        frame.add( painter, c );

        // Layout Status Pane
        c.fill /**/ = GridBagConstraints.HORIZONTAL;
        c.gridwidth = 1;
        c.gridheight = 1;
        c.gridx     = 0;
        c.gridy     = 1;
        c.weightx   = 0.5f;
        c.weighty   = 0.125f;
        frame.add( status, c );

        // Layout Scrollbar
        c.fill /**/ = GridBagConstraints.HORIZONTAL;
        c.gridwidth = 1;
        c.gridheight = 1;
        c.gridx     = 1;
        c.gridy     = 1;
        c.weightx   = 0.125f;
        c.weighty   = 0.125f;
        frame.add( scrollPane, c );
        
        // Arrange UI elements
        frame.pack();
        // Center the frame on screen
        frame.setLocationRelativeTo( null );
        // Display the window.
        frame.setVisible( true );
    }

    public static void run(){
        //Schedule a job for event dispatch thread:
        //creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater( new Runnable() {
            public void run() {
                createAndShowGUI();
            }
        });
    }

    public static void main( String[] args ){
        run();
    }
}

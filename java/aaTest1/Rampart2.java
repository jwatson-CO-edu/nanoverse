




///////// MAIN /////////////////////////////////////////////////////////////////////////////////////

import javax.swing.JFrame;

public class Rampart2 {
    public static void main(String[] args){
        Engine  engine  = new Engine( 25, 25 ); // Game State and Events
        Painter painter = new Painter( engine, 25 );
        int[]  bgnTil = {12,12};
        engine.gen_map( bgnTil, 0.50f, 0.001f);
        // engine.print();

        // Create a JFrame to display the panel
        JFrame frame = new JFrame("Draw Gameboard");

        // Set up the frame
        frame.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );
        frame.setSize( painter.get_width(), painter.get_height() );
        frame.add( painter );
        // Center the frame on screen
        frame.setLocationRelativeTo(null);
        // Make the frame visible
        frame.setVisible(true);
    }
}

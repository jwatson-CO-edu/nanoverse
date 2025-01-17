package org.game;


///////// MAIN /////////////////////////////////////////////////////////////////////////////////////


public class Rampart2 {
    public static void main( String[] args ){

        /// 1. Start Game Engine ///
        Engine  engine  = new Engine( 25, 25 ); // Game State and Events
        int[]   bgnTil  = {12,12};
        engine.gen_map( bgnTil, 0.85f, 0.50f, 0.001f);

        /// 2. Start Graphics ///
        View painter = new View( engine, 25 );

        /// 2. Start User Interface ///
        Controller.createAndShowGUI( engine, painter );
    }
}

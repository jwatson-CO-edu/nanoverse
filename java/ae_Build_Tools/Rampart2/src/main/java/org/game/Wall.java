package org.game;

public class Wall extends ActiveObject {

    Wall( Engine g, int[] addr ){
        super( g, "wall" );
        address = addr.clone();
    }
}

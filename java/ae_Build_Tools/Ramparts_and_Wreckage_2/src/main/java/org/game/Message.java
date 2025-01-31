///////// INIT /////////////////////////////////////////////////////////////////////////////////////

package org.game;



///////// DATA PACKET //////////////////////////////////////////////////////////////////////////////

public class Message {
    // Simplest message container
    public String  topic;
    public GameCmd cmd;
    public String  data;

    Message( String t, GameCmd c, String d ){
        topic = t;
        cmd   = c;
        data  = d;
    }
}

package org.game;

public class Message {
    // Simplest message container
    public String topic;
    public String data;

    Message( String t, String d ){
        topic = t;
        data  = d;
    }
}

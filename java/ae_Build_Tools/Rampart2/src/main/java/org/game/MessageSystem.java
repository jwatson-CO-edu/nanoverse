///////// INIT /////////////////////////////////////////////////////////////////////////////////////

package org.game;


///////// OBSERVER INTERFACE ///////////////////////////////////////////////////////////////////////

public interface MessageSystem {

    void register_observer( String topic, Observer observer);
    void remove_observer( String topic, Observer observer);
    void notify_observers( String topic, String data );

}

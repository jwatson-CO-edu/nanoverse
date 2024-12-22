package aaTest1;

public class ActiveObject extends GameObject {
    protected float health; // Remaining robustness of this object
    protected int   status; // What is this boject doing or feeling?

    ActiveObject( Engine g ){
        super(g);
    }
}






///////// MAIN /////////////////////////////////////////////////////////////////////////////////////

public class Rampart2 {
    public static void main(String[] args){
        Engine engine = new Engine(25, 25); // Game State and Events
        int[]  bgnTil = {12,12};
        engine.gen_map( bgnTil, 0.50f, 0.001f);
        engine.print();
    }
}

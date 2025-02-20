public class StrangeDuck extends BaseDuck {

    public StrangeDuck( String n ){  super(n);  }

    @Override
    public void quack(){
        System.out.println( String.format("Duck named %s, said: QWORK!" , name ));
    }
}

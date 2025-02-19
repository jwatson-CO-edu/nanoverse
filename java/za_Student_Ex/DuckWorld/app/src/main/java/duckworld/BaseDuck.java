public class BaseDuck{
    protected String name;

    public BaseDuck( String n ){  name = n;  }

    public void quack(){
        System.out.println( String.format("Duck named %s, said: QUACK!" , name ));
    }
}

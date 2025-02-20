public class Dog implements Quackable{

    private String name;

    public Dog( String n ){  name = n;  }

    public void quack(){
        System.out.println( String.format("A large, four-legged duck named %s, said: WOOF!" , name ));
    }
}

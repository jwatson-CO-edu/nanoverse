public class Cat implements Quackable{

    private String name;

    public Cat( String n ){  name = n;  }

    public void quack(){
        System.out.println( String.format("A small, four-legged duck named %s, said: MEOW!" , name ));
    }
}

import java.util.ArrayList;

public class Main {
    public static void main(String[] args) {

        // Create arrs
        ArrayList<BaseDuck>  base = new ArrayList<BaseDuck>();
        ArrayList<Quackable> quak = new ArrayList<Quackable>();

        // Populate arrs
        base.add( new BaseDuck( "Linda" ) );
        base.add( new StrangeDuck( "Paul" ) );
        quak.add( new Dog( "Spot" ) );
        quak.add( new Cat( "Malgantor the World-Eater" ) );

        // Observe Behavior
        System.out.println("##### Welcome to Duck Simulator 2025 #####\nYou have 30 days left in your Free Trial!\n");
        for( BaseDuck  d : base ){  d.quack();  }
        for( Quackable d : quak ){  d.quack();  }
    }
}



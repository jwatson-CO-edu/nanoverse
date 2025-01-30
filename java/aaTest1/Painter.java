import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class Painter extends JPanel implements ActionListener {

    protected Engine game; // Container
    private   int    unitPx;
    private   int    Wwin, Hwin;
    private   Timer  timer;

    public int get_width(){   return Wwin;  }
    public int get_height(){  return Hwin;  }

    public Painter( Engine g, int gridUnit_px ) {
        // Connect the renderer to the game engine
        game   = g;
        unitPx = gridUnit_px;
        Wwin   = game.Wmap * unitPx;
        Hwin   = game.Hmap * unitPx;
        // Set up a timer to trigger actionPerformed periodically
        timer = new Timer( 24, this );
        timer.start();
    }

    public void paint_terrain( Graphics2D g2d ){
        for( int j = 0 ; j < game.Wmap ; j++ ){
            for( int i = 0 ; i < game.Hmap ; i++ ){
                g2d.setColor( game.tileMap[j][i].clr );
                g2d.fillRect( j*unitPx, Hwin-(i+1)*unitPx, unitPx, unitPx    );
            }
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Cast Graphics to Graphics2D for advanced features
        Graphics2D g2d = (Graphics2D) g;

        // Set the background color to black
        g2d.setColor( Color.BLACK );
        g2d.fillRect( 0, 0, Wwin, Hwin );
        paint_terrain( g2d );
    }

    @Override
    public void actionPerformed( ActionEvent e ) {
        // Repaint the panel to show the updated position
        repaint();
    }

}

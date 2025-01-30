import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class AnimatedRectangle extends JPanel implements ActionListener {

    private int rectWidth = 200;
    private int rectHeight = 100;
    private int x = -1;
    private int y = -1;
    private int dx = 2;
    private int dy = 2;
    private Timer timer;

    public AnimatedRectangle() {
        // Set up a timer to trigger actionPerformed periodically
        timer = new Timer(24, this);
        timer.start();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Cast Graphics to Graphics2D for advanced features
        Graphics2D g2d = (Graphics2D) g;

        // Set the background color to black
        g2d.setColor(Color.BLACK);
        g2d.fillRect(0, 0, getWidth(), getHeight());

        // Initialize rectangle position if not already set
        if (x == -1 && y == -1) {
            x = (getWidth() - rectWidth) / 2;
            y = (getHeight() - rectHeight) / 2;
        }

        // Set the rectangle color to red
        g2d.setColor(Color.RED);

        // Draw the rectangle
        g2d.fillRect(x, y, rectWidth, rectHeight);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        // Update rectangle position
        x += dx;
        y += dy;

        // Check for collision with panel edges and reverse direction if needed
        if (x <= 0 || x + rectWidth >= getWidth()) {
            dx = -dx;
        }
        if (y <= 0 || y + rectHeight >= getHeight()) {
            dy = -dy;
        }

        // Repaint the panel to show the updated position
        repaint();
    }

    public static void main(String[] args) {
        // Create a JFrame to display the panel
        JFrame frame = new JFrame("Bouncing Red Rectangle on Black Background");

        // Set up the frame
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(400, 300);

        // Add the custom panel to the frame
        AnimatedRectangle panel = new AnimatedRectangle();
        frame.add(panel);

        // Center the frame on screen
        frame.setLocationRelativeTo(null);

        // Make the frame visible
        frame.setVisible(true);
    }
}

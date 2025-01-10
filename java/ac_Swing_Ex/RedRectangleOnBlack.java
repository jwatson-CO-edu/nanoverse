import javax.swing.*;
import java.awt.*;

public class RedRectangleOnBlack extends JPanel {

    // Method to set up the panel's appearance
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Cast Graphics to Graphics2D for advanced features
        Graphics2D g2d = (Graphics2D) g;

        // Set the background color to black
        g2d.setColor(Color.BLACK);
        g2d.fillRect(0, 0, getWidth(), getHeight());

        // Set the rectangle color to red
        g2d.setColor(Color.RED);

        // Define rectangle's position and size
        int rectWidth = 200;
        int rectHeight = 100;
        int x = (getWidth() - rectWidth) / 2; // Center horizontally
        int y = (getHeight() - rectHeight) / 2; // Center vertically

        // Draw the rectangle
        g2d.fillRect(x, y, rectWidth, rectHeight);
    }

    public static void main(String[] args) {
        // Create a JFrame to display the panel
        JFrame frame = new JFrame("Red Rectangle on Black Background");

        // Set up the frame
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(400, 300);

        // Add the custom panel to the frame
        RedRectangleOnBlack panel = new RedRectangleOnBlack();
        frame.add(panel);

        // Center the frame on screen
        frame.setLocationRelativeTo(null);

        // Make the frame visible
        frame.setVisible(true);
    }
}

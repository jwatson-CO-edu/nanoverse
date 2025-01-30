package ab_OpenGL_Ex;

import com.jogamp.opengl.*;
import com.jogamp.opengl.awt.GLCanvas;
import com.jogamp.opengl.util.Animator;

import javax.swing.*;

public class RedTriangle implements GLEventListener {

    @Override
    public void init(GLAutoDrawable drawable) {
        // Initialization code, called once when the context is created
        GL2 gl = drawable.getGL().getGL2();
        gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set black as the clear color
    }

    @Override
    public void dispose(GLAutoDrawable drawable) {
        // Resource cleanup code, called when the context is destroyed
    }

    @Override
    public void display(GLAutoDrawable drawable) {
        // Called to draw the scene
        GL2 gl = drawable.getGL().getGL2();
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT); // Clear the screen

        // Draw the red triangle
        gl.glBegin(GL2.GL_TRIANGLES);
        gl.glColor3f(1.0f, 0.0f, 0.0f); // Set the color to red
        gl.glVertex2f(-0.5f, -0.5f); // Bottom left
        gl.glVertex2f(0.5f, -0.5f);  // Bottom right
        gl.glVertex2f(0.0f, 0.5f);   // Top
        gl.glEnd();
    }

    @Override
    public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {
        // Adjust the viewport based on window resize
        GL2 gl = drawable.getGL().getGL2();
        gl.glViewport(0, 0, width, height);
    }

    public static void main(String[] args) {
        // Create a GLCanvas to render OpenGL content
        GLProfile profile = GLProfile.get(GLProfile.GL2);
        GLCapabilities capabilities = new GLCapabilities(profile);
        GLCanvas canvas = new GLCanvas(capabilities);

        // Create the event listener
        RedTriangle redTriangle = new RedTriangle();
        canvas.addGLEventListener(redTriangle);

        // Create a JFrame to hold the canvas
        JFrame frame = new JFrame("Red Triangle with JOGL");
        frame.getContentPane().add(canvas);
        frame.setSize(800, 600);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);

        // Start an animator to repaint the canvas continuously
        Animator animator = new Animator(canvas);
        animator.start();
    }
}

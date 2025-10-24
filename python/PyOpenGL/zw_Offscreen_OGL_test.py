from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import numpy as np

def render_wireframe_cube_headless(output_file='cube.jpg', size=(800, 600), rotation=(30, 45)):
    """
    Render a blue wireframe cube to JPG using PyOpenGL with hidden GLUT window.
    
    Parameters:
    -----------
    output_file : str
        Output filename for the JPG
    size : tuple
        Image size (width, height)
    rotation : tuple
        Rotation angles in degrees (x_rotation, y_rotation)
    """
    width, height = size
    
    # Initialize GLUT
    glutInit()
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    glutInitWindowSize(width, height)
    glutCreateWindow(b"Offscreen")
    glutHideWindow()  # Hide the window
    
    # Set up OpenGL state
    glViewport(0, 0, width, height)
    glClearColor(1.0, 1.0, 1.0, 1.0)  # White background
    
    # Set up projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width / height, 0.1, 50.0)
    
    # Set up modelview
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -3.0)
    glRotatef(rotation[0], 1, 0, 0)
    glRotatef(rotation[1], 0, 1, 0)
    
    # Enable antialiasing
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glLineWidth(2.0)
    
    # Clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    # Define cube vertices
    vertices = [
        [-0.5, -0.5,  0.5],
        [ 0.5, -0.5,  0.5],
        [ 0.5,  0.5,  0.5],
        [-0.5,  0.5,  0.5],
        [-0.5, -0.5, -0.5],
        [ 0.5, -0.5, -0.5],
        [ 0.5,  0.5, -0.5],
        [-0.5,  0.5, -0.5],
    ]
    
    # Define edges
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Front
        (4, 5), (5, 6), (6, 7), (7, 4),  # Back
        (0, 4), (1, 5), (2, 6), (3, 7),  # Connecting
    ]
    
    # Draw cube
    glColor3f(0.2, 0.5, 1.0)  # Blue color
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()
    
    # Flush
    glFlush()
    
    # Read pixels
    glPixelStorei(GL_PACK_ALIGNMENT, 1)
    data = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
    
    # Convert to image
    image = Image.frombytes("RGB", (width, height), data)
    image = image.transpose(Image.FLIP_TOP_BOTTOM)
    
    # Save as JPG
    image.save(output_file, 'JPEG', quality=95)
    print(f"Saved wireframe cube to {output_file}")

if __name__ == "__main__":
    for i in range( 10 ):
        render_wireframe_cube_headless(f'data/cube{i}.jpg', size=(800, 600), rotation=(5*i, 15*i))
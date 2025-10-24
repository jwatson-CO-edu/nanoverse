import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from PIL import Image

def display_wireframe_cube(save_frame=False, output_file='cube.jpg'):
    """Display a blue wireframe cube using PyOpenGL."""
    
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
    
    # Initialize pygame and OpenGL
    pygame.init()
    display = (800, 600)
    screen = pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Blue Wireframe Cube")
    
    # Set up perspective
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -3)
    
    # Enable antialiasing
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glLineWidth(2.0)
    
    # Main loop
    clock = pygame.time.Clock()
    rotation_x = 0
    rotation_y = 0
    frame_saved = False
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    # Save frame when spacebar is pressed
                    save_opengl_frame(display, output_file)
                    print(f"Frame saved to {output_file}")
        
        # Auto-rotate
        rotation_x += 0.5
        rotation_y += 0.3
        
        # Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # Apply rotation
        glPushMatrix()
        glRotatef(rotation_x, 1, 0, 0)
        glRotatef(rotation_y, 0, 1, 0)
        
        # Draw cube edges in blue
        glColor3f(0.2, 0.5, 1.0)  # Blue color
        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()
        
        glPopMatrix()
        
        # Update display
        pygame.display.flip()
        
        # Save frame on first render if requested
        if save_frame and not frame_saved:
            save_opengl_frame(display, output_file)
            print(f"Frame saved to {output_file}")
            frame_saved = True
            running = False  # Exit after saving
        
        clock.tick(60)
    
    pygame.quit()

def save_opengl_frame(display_size, filename='cube.jpg'):
    """Save the current OpenGL frame as a JPG image."""
    width, height = display_size
    
    # Read pixels from OpenGL
    glPixelStorei(GL_PACK_ALIGNMENT, 1)
    data = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
    
    # Create PIL Image
    image = Image.frombytes("RGB", (width, height), data)
    
    # Flip vertically (OpenGL coordinates are bottom-up)
    image = image.transpose(Image.FLIP_TOP_BOTTOM)
    
    # Save as JPG
    image.save(filename, 'JPEG', quality=95)

if __name__ == "__main__":
    # Option 1: Run and press SPACEBAR to save a frame
    display_wireframe_cube()
    
    # Option 2: Save one frame and exit immediately
    # display_wireframe_cube(save_frame=True, output_file='cube.jpg')
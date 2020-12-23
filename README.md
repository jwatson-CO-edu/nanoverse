# nanoverse
Smol 3D for Linux
### Dev Plan
```
/***** Architecture 1: Making Triangles Dance *****/
 ~  Functional Core
[Y] Find/Compile VBO Example - 2020-12-23 , http://www.songho.ca/opengl/gl_vbo.html , vboSimple.zip
[ ] Context creation process w/ defaults
    [Y] Default params - 2020-12-23 , Struct with default params
    [ ] Default draw
    [ ] Default resize
     ~  No default interaction
[ ] Cube/Cuboid primitive mesh
[ ] Cube/Cuboid primitive model
    [ ] Vertex buffer
    [ ] How does example do color?
    [ ] Draw
    [ ] Investigate facet optimization (Search: "OpenGL index buffer")
[ ] Entity Base Class
    [ ] Draw mesh
    [ ] Draw model
        [ ] Simple Model
        [ ] Composite Model (Merge buffers for speed?(Inherit Model))
    [ ] Pose & Motion (Simplicity: Both static and dynamic objects inherit this)
[ ] Camera class
    [ ] Pose & Motion
[ ] Shader Experiment(s)
    [ ] CAD-style outlines (Vertex shader?) with flat colors and simplest shading
        [ ] Sharp edges
        [ ] Object-background edge
    [ ] Cel Shader, Unsure of what kind of shader does what I want
        [ ] Search: "OpenGL cel toon shader vertex shader"
        [ ] Search: "OpenGL cel toon shader fragment shader"
    [ ] Investigate using outlines and cel shading simultaneously
    [ ] Nvidia Test
    [ ] Intel NUC Test
    [ ] Add shader slots to Model class
[ ] SDL2 Interaction
    [ ] KB input
    [ ] Mouse input
    [ ] JS input
[ ] Player Class
    [ ] FPV Camera
    [ ] KB input
    [ ] Mouse input
    [ ] JS input
[ ] Flying Camera
    [ ] "Descent" ownship motion 
        [ ] Assumed neutral bouyancy (levitate)
        [ ] Damped stop after key-up
    
/***** Architecture 2: Scenes *****/
 ~  Have Fun
 ~  NOTE: The entire scene is always loaded
[ ] Scene Graph
    [ ] Subtree complexity measure
    [ ] Subtree drawing
[ ] Agent Class
    [ ] State exchange format (Graphs? Children hashed?)
    [ ] Slot for brain
    [ ] Boids
[ ] Add Bullet Physics
    [ ] Install/import?
    [ ] Bullet Model Properties
        [ ] Collision mesh
        [ ] Collision Model
        [ ] Intrinsic/Extrinsic Properties
    [ ] Subtree solvers
[ ] Player Class Extension
    [ ] Target selection test 
        [ ] Investigate Bullet ray casting
            [ ] If not, use graphics class implementation
[ ] Bullet 4x4
    [ ] Terrain class
    [ ] Wheel contact: Clip/Support
    [ ] Wheel friction model
    [ ] Obstacle collision
[ ] Tank
    [ ] Decide First/Third PV
    [ ] Independent Aim/Steer

/***** Architecture 3: Advanced Simulation *****/
 ~  Fast & Parallel
[ ] Dynamic and Optimized Loading
    [ ] Investigate a loading thread
    [ ] Subtree loading decisions
    [ ] Investigate loading thread w/ queue
[ ] Destructible Entities
    [ ] Pre-compute fractures
    [ ] Investigate comput shaders
        [ ] OpenCL
        [ ] OpenGL
        [ ] Decide which is { faster to run, easier to write }
[ ] Learning Agents & Self-Play
    [ ] Non-rendered simulation
    [ ] Agent creation manager
[ ] Network play
    [ ] Multiple players
    [ ] Hosted world
    [ ] Agent exchange

/***** Architecture 4: A Sim for All Seasons *****/
 ~  Share what you made
[ ] ROS2 Integration
    [ ] Load URDF
    [ ] Robot State
    [ ] Joint State
    [ ] Controller interface
[ ] Python API
[ ] Golang API
[ ] Publish
    [ ] Linux .deb distribution
        [ ] How to handle dependencies?
    [ ] Python package
    [ ] Golang package
    [ ] Add to Ubuntu multiverse
```
### Dependencies
#### OpenGL
1. `sudo add-apt-repository ppa:oibaf/graphics-drivers`
2. `sudo apt update`
3. `sudo apt install mesa-utils freeglut3-dev`
#### Eigen
1. Download Eigen3: http://eigen.tuxfamily.org/index.php?title=Main_Page  
(Then navigate to the director to where it was downloaded)  
1. `mkdir build && cd $_`
2. `cmake ..`
3. `sudo make install`
# nanoverse
A rendering engine for artistic projects and lightweight simulations
* Keep work on this toy to a responsible minimum
* All units in meters `[m]`


### Dev Plan 1: PhD Treats (Tentatively useful)
```
/***** Jupyter 1: Making Triangles Dance *****/
[ ] Animated ship on starfield
    [ ] Fix Phong lighting
[ ] Controllable ship on grid field
    [ ] Pygame needed for controller input?  Does it work in Jupyter?


/***** Jupyter 2: Research Applications *****/
[ ] Recreate UR5 model from Graphics Class
    [ ] Investigate pythreejs shape primitives rather than rewriting your own!
[ ] Put text in the 3D environment
[ ] Wrap the setup into functions --> module file
[ ] Recreate reference frame object


/***** Jupyter 3: Robot state *****/
[ ] Query robot and render joint state
[ ] Query sensor and render FT state
[ ] Render gripper state


/***** Rendering 1: Research Applications *****/
[ ] Textured object
[ ] Textured dinosaur (Graphics Class)


/***** Rendering 2: Restore *****/
[ ] Render Restorebot data
    [ ] Point cloud
    [ ] Stretch RGB over point cloud (Graphics Class)
    { } Fuse pictures
[ ] Render a ROS sensor frame in Jupyter
```


### Dev Plan 2: Pure Frivolity (Dangerous)
```
/***** Architecture 1: Mach Test (Zig) *****/
 ~  Functional Core
[ ] Cube/Cuboid primitive model
    [ ] Vertex array rendering
        [ ] Set up light(s)
        [ ] Transform Camera
    [ ] Vertex buffer rendering
    [ ] Investigate facet optimization (Search: "OpenGL index buffer")
[ ] Camera class
    [ ] Pose & Motion: be able to change position, target, and up-vec independently with others staying the same
[ ] Shader Experiment(s)
    [ ] CAD-style outlines (Vertex shader?) with flat colors and simplest shading
        [ ] Sharp edges
            * https://gamedev.stackexchange.com/a/86413, What does this do at the interface between flat triangles?
            * https://en.wikibooks.org/wiki/GLSL_Programming/Unity/Toon_Shading#Outlines
        [ ] Object-background edge
            * https://www.codeproject.com/Articles/8499/Generating-Outlines-in-OpenGL
        [ ] https://github.com/nvpro-samples/gl_cadscene_rendertechniques
    [ ] Cel Shader, Unsure of what kind of shader does what I want
        [ ] https://www.lighthouse3d.com/tutorials/glsl-12-tutorial/toon-shading-version-iii/
        [ ] https://stackoverflow.com/a/5796168
        [ ] http://rbwhitaker.wikidot.com/toon-shader
        [ ] http://www.cs.sjsu.edu/~bruce/fall_2016_cs_116a_lecture_glsl_opengl_shading_language_part_1_of_2.html
    [ ] Investigate using outlines and cel shading simultaneously
    [ ] Add shader slots to Model class
[ ] Flying Camera: "Descent" ownship motion 
    [ ] Mouse: { X: Yaw , Y: Pitch }
    [ ] KB: { W/S: Fwd/Bck , A/D: Strafe }
    [ ] JS: { Left  Stick: { X: ??? , Y: ??? } , 
              Right Stick: { X: ??? , Y: ??? } ,
              Buttons: { ??? }                 }
    [ ] Assumed neutral bouyancy (levitate)
    { } Damped stop after key-up    


/***** Architecture 2: Frames for Fun *****/
 ~  Purpose: Digital sculpture platform with a dash of simulation
 ~  Be careful with your time
[ ] Spheres (Take from CU graphics/MS)
[ ] N-body simulation, orbiting spheres
[ ] L-system tree
[ ] Mesh with animated shader
[ ] Load ship models into space scene
[ ] Glitch shader
[ ] VHS Shader
[ ] Particle shader


/***** Architecture 3: Let's Code (Applied) Physics *****/ 
 ~  Purpose: Daydream itch
[ ] Find/Create Clifford lib in Zig
[ ] Clifford projectile
[ ] Clifford planets
[ ] Clifford electrodynamics
[ ] Electrodynamics
    [ ] Magnets
        [ ] Motor field optimizer
        [ ] Plasma confinement optimizer
    [ ] Generative 


/***** Architecture 4: Advanced Simulation *****/
 ~  Fast & Parallel
[ ] Modeling software integration
    [ ] Parse Blender OBJ format (textured?)
    [ ] Parse popular CAD format
[ ] Advanced memory management: Dynamic and Optimized Un/Loading
    [ ] Investigate a loading thread
    [ ] Investigate background loading
    [ ] Subtree/chunk loading decisions like minecraft
    [ ] Decide on a "chunk size"
    [ ] Investigate loading thread w/ queue
    [ ] Garbage collection thread: Can I delay destruction of a shared pointer with no references?
    [ ] Investigate unloading thread w/ queue
[ ] Investigate compute shaders
    [ ] OpenCL
    [ ] OpenGL
    [ ] Decide which is { faster to run, easier to write }
[ ] Destructible Entities
    [ ] Bullet Stress Test, # of Physics Entities at 60 fps
        [ ] Nvidia GPU
        [ ] Intel  NUC
        [ ] Decide on # of allowed physics entities
    [ ] Pre-compute fractures
[ ] Investigate hierarchical collisition detections
    [ ] Bullet should do this?
    [ ] If not, then use RAPID (MS Thesis code)
[ ] Learning Agents & Self-Play
    [ ] Non-rendered simulation
    [ ] Agent creation manager
[ ] Advanced shading
    [ ] Subdivide drawing mesh for more appealing shading: https://www.cs.ucr.edu/~craigs/courses/2016-fall-cs-130/lab-09.html
    [ ] Special Topics
        [ ] cube map arrays
        [ ] Scrape class notes
    [ ] Evaluate lighted projectiles
    [ ] Investigate shadow casting
    [ ] Investigate ray tracing: Does OpenGL take advantage of ray tracing hardware?


/***** Architecture 5: A Sim for All Seasons *****/
 ~  Share what you made
[ ] ROS2 Integration
    [ ] Load URDF
    [ ] Robot State
    [ ] Joint State
    [ ] Controller interface
[ ] Network play
    [ ] Multiple players
    [ ] Hosted world
    [ ] Agent exchange
[ ] Live Coding IDE
[ ] Publish
    [ ] Linux .deb distribution
        [ ] How to handle dependencies?
    [ ] Python package
    [ ] Golang package
    [ ] Add to Ubuntu multiverse
```


### Dev Plan 3: Pure Folly (Do not attempt before establishing career!)
```
/***** Architecture N: Shall?  we?  play?  a?  game? *****/
 ~  Have Fun
 ~  NOTE: The entire scene is always loaded
[ ] Evaluate Modular Refactor:
    [ ] Consider Interfaces for each kind of activity: { Graphics, Control, Intelligence, More? }
    [ ] Does it make sense to have an interface for each?
    [ ] Does an interface imply a plugin interface?
    [ ] If using plugins, Connect at interface in a way that reduces pointer math and communication overhead
        For example, Using multiple interfaces handled with an inheritance list 
        that facilitates information flow through pointers on both sides such that s/getters are never used?
        https://www.arbinada.com/en/node/1466
[ ] Player Class (Is this already supported in Mach?)
    [ ] FPV Camera
    [ ] KB input
    [ ] Mouse input
    [ ] JS input
[ ] Scene Graph (Is this already supported in Mach?)
    [ ] Subtree complexity measure
    [ ] Subtree drawing
[ ] Evaluate Entity Component System (Is this already supported in Mach?)
    [ ] Different components for { Drawing, Mesh, Physics }
    [ ] Will the capabilities of objects really be that different?
    [ ] Entity Base Class
        [ ] Draw mesh
        [ ] Draw model
            [ ] Simple Model
            [ ] Composite Model (Merge buffers for speed?(Inherit Model))
        [ ] Pose & Motion (Simplicity: Both static and dynamic objects inherit this)
[ ] Agent Class (Is this already supported in Mach?)
    [ ] State exchange format (Graphs? Children hashed?)
    [ ] Slot for brain
    [ ] Advanced Boids Demo
[ ] Add Bullet Physics (Is this already supported in Mach?)
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
    [ ] Independent Aim/Steer (w/ dual views!)

```
# Dependencies
## OpenGL
1. `sudo add-apt-repository ppa:oibaf/graphics-drivers`
2. `sudo apt update`
3. `sudo apt install mesa-utils freeglut3-dev`
## Nim
1. `nimble install https://github.com/nimgl/opengl.git`
2. `nimble install https://github.com/nimgl/glfw.git`
## Eigen
1. Download Eigen3: http://eigen.tuxfamily.org/index.php?title=Main_Page  
(Then navigate to the director to where it was downloaded)  
1. `mkdir build && cd $_`
2. `cmake ..`
3. `sudo make install`

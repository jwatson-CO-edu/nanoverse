# nanoverse
A rendering engine for jupyter visualization and artistic projects
* Keep work on this toy to a responsible minimum
* All units in meters `[m]`


### Dev Plan 1: PhD Treats (Tentatively useful)
```
/***** Jupyter 1: Making Triangles Dance, SKIPPED *****/
[N] Animated ship on starfield - 2022-07-29: pythreejs is NOT made for programmatic animation
    [N] Fix Phong lighting
[N] Controllable ship on grid field - 2022-07-29: pythreejs is NOT made for programmatic animation
    [N] Pygame needed for controller input?  Does it work in Jupyter?


/***** Jupyter 2: Research Applications *****/
[ ] Recreate UR5 model from Graphics Class
    [Y] Investigate pythreejs shape primitives rather than rewriting your own! - 2022-08-04: Links {1,2,3} drawn with p3js cylinders + wireframe!
    [ ] Port robot drawing procedure from C++ graphics assignment
    [ ] Draw TCP frame (see ref frame obj)
[ ] Recreate reference frame object
    [ ] Put text in the 3D environment
    [ ] Label axes
[ ] Can I animate simply by calling render repeatedly?: https://threejs.org/docs/#manual/en/introduction/Creating-a-scene
[Y] Wrap the setup into functions - 2022-08-04: Simplest rendering function for display and debugging
[Y] Animation - 2022-08-09: Simplest animation is changing a field and re-rendering the frame, but
                            in this scheme there is no buffering, so it is very slow and jumpy


/***** Jupyter 3: Robot state *****/
[ ] Query robot and render joint state
[ ] Port FT drawing code from robot manip class (open3d --to-> pythreejs)
[ ] Query sensor and render FT state
[ ] Render gripper state (simply)


/***** Rendering 1: Restore *****/
[ ] Render Restorebot data
    [ ] Point cloud
    [ ] Textured object
    [ ] Textured dinosaur (Graphics Class)
    [ ] Stretch RGB over point cloud (Graphics Class)
    { } Fuse pictures
[ ] Render a ROS sensor frame in Jupyter


/***** Rendering 2: Restore *****/
[ ] Draw RestoreBot
[ ] Draw RestoreCart2 relative to RestoreBot
```


### Dev Plan 2: Pure Frivolity (Dangerous!)
```
/***** Architecture 1: Dlang + RayLib *****/
 ~  Purpose: Let yourself have some graphics, as a treat!

[>] Grid Glider - Delta Glider
    [Y] Glider with planelike motion - 2022-09-19: Really need to calm down on the graphics, you don't need them!
    [Y] Glider flies over flat XY grid - 2022-09-22: Smooth flight and pleasing grid that fills the draw distance w/o slowdown
    [Y] Camera follow glider - 2022-09-22: Camera stuck fast to glider!
    [Y] Ship keyboard steering with camera follow - 2022-09-23: Very easy!
    [Y] Test joystick input - 2022-12-04: If the gamepad does not work, it may not be on the list!, https://github.com/gabomdq/SDL_GameControllerDB/blob/master/gamecontrollerdb.txt
    [Y] Player glider with joystick input, 2022-12-06: Easy!
    [Y] Simple lighting and Flat shading, 2022-12-01: Ambient lighting is probably enough
    [Y] Visually pleasing contrail ribbon, 2022-12-05: Timer was the tricky part.
        [Y] Triangle strip, 2022-12-05: Timer was the tricky part.
        [Y] Alpha fades to nothing at trailing end, 2022-12-05: Timer was the tricky part.
        [N] Shine fades to nothing at trailing end, 2022-12-05: Unsure how to apply shader to a single triangle
    [Y] Dragging camera (Starfox), 2022-12-06: Pleasing to look at!
    [Y] Randomly populate grid with filled squares, 2022-12-06: Adds visual interest
    [Y] Floating triangle debris, 2022-12-06: Easy!
    [>] Video -or- animated GIF output --to-> IG

 * Context switch to FINCH

[ ] Piped Ribbons - Generative art
    [ ] Randomly trace a 3D grid, with 90deg curved turn limit with grid radius
    [ ] Multiple, random non-colliding ribbons
    [ ] Multiple, random non-colliding ribbons in a loop and orbiting camera
    [ ] Video -or- animated GIF output --to-> IG

 * Context switch to FINCH

[ ] Terrain generation - Delta Glider
    [ ] Perlin Noise grid
    [ ] Marching cubes
    [ ] Glider collision with landscape
    [ ] Video -or- animated GIF output --to-> IG

 * Context switch to FINCH

[ ] Engineering Center FROM SPACE - Generative art
    [ ] Generative angular castle (A la EC)
    [ ] Windows
    [ ] Textures
    [ ] Rotating "turrets"
    [ ] Friggin' laser beams
    { } Fends off Delta Gliders?
        { } EXPLOSIONS ?!?
    [ ] Loop
    [ ] Video -or- animated GIF output --to-> IG

 * Context switch to FINCH

[ ] Landscape generation - Delta Glider
    [ ] Trees
    [ ] Grass
    [ ] Water (Simplest)
    [ ] Floating islands
    [ ] Video -or- animated GIF output --to-> IG

 * Context switch to FINCH

[ ] Tetrahedra - Generative Art
    [ ] ???

 * Context switch to FINCH

/***** Architecture 2: Frames for Fun *****/
 ~  Purpose: Digital sculpture platform with a dash of simulation
 ~  Be careful with your time
[ ] N-body simulation, orbiting spheres
[ ] L-system tree
[ ] Mesh with animated shader
[ ] Load ship models into space scene
[ ] Glitch shader
[ ] VHS Shader
[ ] Particle shader


/***** Architecture 3: Let's Code (Applied) Physics *****/ 
 ~  Purpose: Daydream itch
[ ] Find/Create Clifford lib in Dlang
[ ] Clifford projectile
[ ] Clifford planets
[ ] Clifford electrodynamics
[ ] Electrodynamics
    [ ] Magnets
        [ ] Motor field optimizer
        [ ] Plasma confinement optimizer
    [ ] Generative 


/***** Architecture 4: Advanced Simulation *****/
 ~  Purpose: Play with a 3D game engine (Godot)
 ~  Fast & Parallel
[ ] A Peaceful Drive (My Ideal PC Game)
    [ ] Terrain and Landscape generation
    [ ] Choose physics engine
    [ ] Glider with satisfying dynamics and controls
    [ ] Rover with satisfying dynamics and controls
    [ ] Glider <--> Rover transformation
    [ ] Save game w/ persistent landscape
    [ ] Marker system
    [ ] Screenshots 
    [ ] Steam integration
        [ ] Savegames
        [ ] Screenshots
        [ ] World sharing
        [ ] Co-op play
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
[ ] FINCH Integration
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
# Resources


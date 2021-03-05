# nanoverse
Minimum Viable 3D for Linux
* All units in meters `[m]`

### Dev Plan
```
/***** Architecture 1: Making Triangles Dance *****/
 ~  Functional Core
[Y] Find/Compile VBO Example - 2020-12-23 , http://www.songho.ca/opengl/gl_vbo.html , vboSimple.zip
[Y] Context creation process w/ defaults >> COMPLETE - 2021-03-04
    [Y] Default params - 2020-12-23 , Struct with default params
    [Y] Default draw - 2020-12-23
    [Y] Default resize - 2020-12-23
     ~  No default interaction
    [Y] Default Demo window - Should be an example how things are rendered. - 2021-03-04
[ ] Cube/Cuboid primitive mesh: Parameterize the Songho cube
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
[ ] Evaluate Modular Refactor:
    [ ] Consider Interfaces for each kind of activity: { Graphics, Control, Intelligence, More? }
    [ ] Does it make sense to have an interface for each?
    [ ] Does an interface imply a plugin interface?
    [ ] If using plugins, Connect at interface in a way that reduces pointer math and communication overhead
        For example, Using multiple interfaces handled with an inheritance list 
        that facilitates information flow through pointers on both sides such that s/getters are never used?
        https://www.arbinada.com/en/node/1466
[ ] Player Class
    [ ] FPV Camera
    [ ] KB input
    [ ] Mouse input
    [ ] JS input
[ ] Flying Camera: "Descent" ownship motion 
    [ ] Mouse: { X: Yaw , Y: Pitch }
    [ ] KB: { W/S: Fwd/Bck , A/D: Strafe }
    [ ] JS: { Left  Stick: { X: ??? , Y: ??? } , 
              Right Stick: { X: ??? , Y: ??? } ,
              Buttons: { ??? }                 }
    [ ] Assumed neutral bouyancy (levitate)
    { } Damped stop after key-up    
[ ] Evaluate Boost Interop: Time to get modern!
    [ ] Status of Boost OGL Support   & Evaluate refactor
    [ ] Status of Boost Eigen Support & Evaluate refactor
[ ] Evaluate drawing pipeline optimization
    [ ] Find out where VBO and other graphic memory is stored
    [ ] Buffers contiguous in memory?

/***** DECISION POINT *****/
[[ ]] WHAT IS THIS SOFTWARE? ~ WHAT WOULD DR. WATSON DO?
      1. Benefits
      2. Needs
      3. Existing expertise
      4. Is there an easy alternative?
      5. Are there { stable, widely-used, good-quality, C++ } libs that can prevent massive wheel-inventing?
      CHOOSE SUBSET -and- ASSIGN PRIORITY:
      
      /** Renderers **/
       * Cost/Benefit
            + Looks cool
            + Minimal scope
            - Competes with research projects
       * Scope & Prerequisites
            ~ Abstraction of the OpenGL rendering pipeline
            ~ Opportunity to limit scope to short-term projects
      [ ] Generative Art Program
      [ ] Scientific Visualization Package
      
      /** Simulators **/
       * Cost/Benefit
            - Rendering pipeline is only a small part
            - Functional Core is potentially massive
            + Experience from MS thesis
            - Competes with OnShape development effort
            + Scratches the itch
            - Relies on domain knowledge in areas I have not looked at in a long time
            - Poor simulation fidelity: 
                    - This is a research area outside my own
                    - Suitable for course/hobby grade design work ONLY
       * Scope & Prerequisites
            ~ { simulation, event } management
            ~ Separation of rendering and simulation
      [ ] Robotics/AI/Swarm simulation platform
      [ ] 3D Game Engine
       * ADDITIONAL Scope & Prerequisites
            ~ Careful attention to numeric stability
              Consideration and testing of integration techniques (UT Sim. Graphics)
            ~ Domain knowledge
      [ ] Engineering Simulation Suite
      
      /** All-in-One **/
       * Cost/Benefit
            + Will provide { reward, experience } for years
            + Meets multiple side project goals
            + Broadest possible community
            - MASSIVE { time, effort } investment required
            - Competes with widely-used packages developed by large teams
            - Difficult
       * Scope & Prerequisites
            ~ ALL OF THE ABOVE
[[ ]] ORGANIZE THE REMAINING DEV PLAN ACCORDINGLY
      [ ] Move pruned subgoals to de-prioritzed projects


/***** Architecture 2: Scenes & Interactions *****/
 ~  Have Fun
 ~  NOTE: The entire scene is always loaded
[ ] Scene Graph
    [ ] Subtree complexity measure
    [ ] Subtree drawing
[ ] Evaluate Entity Component System
    [ ] Different components for { Drawing, Mesh, Physics }
    [ ] Will the capabilities of objects really be that different?
[ ] Agent Class
    [ ] State exchange format (Graphs? Children hashed?)
    [ ] Slot for brain
    [ ] Boids Demo
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
[ ] Modeling software integration
    [ ] Parse Blender OBJ format (textured?)
    [ ] Parse popular CAD format
[ ] Serialization (Use Boost?)
    [ ] Entities / Agents: Store params
    [ ] Scene graphs: Make reconstructing the graph EASY
        [ ] Terrain: How to chunk?
        [ ] Mark agents in/active
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


/***** Architecture 4: A Sim for All Seasons *****/
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
[ ] Python API
[ ] Golang API
[ ] Publish
    [ ] Linux .deb distribution
        [ ] How to handle dependencies?
    [ ] Python package
    [ ] Golang package
    [ ] Add to Ubuntu multiverse


/***** Architecture 5: Let's Code (Applied) Physics *****/    
[ ] FEA / Beam Elements: Compute solver (See above decision)
    [ ] Voxelize: Soft robots , https://www.creativemachineslab.com/voxcad.html
    [ ] Dynamic Truss Strain & Fracture
    [ ] Deformable objects
[ ] Sound propagation
    [ ] Folded-horn Waveguide for Subwoofer
[ ] Fluid dynamics
[ ] Electrodynamics
    [ ] Magnets
        [ ] Motor field optimizer
        [ ] Plasma confinement optimizer
    [ ] Generative 
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
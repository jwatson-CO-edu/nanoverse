# nanoverse
Artistic projects rendered in various frameworks and engines, written in (and grouped by) various languages 
* Keep work on this toy to a responsible minimum
* All units in meters `[m]`


## Architecture 1: C++ & RayLib  (Dangerous!)
* Purpose: Let yourself have some graphics, as a treat!  

* `[>]` BoxKart
    - `[Y]` Simple geometry, 2023-09-26: Fixed an error that caused all displacements to be double-counted
    - `[Y]` Tank steering with gamepad, 2023-09-27: Got it on the first try!
        * `[Y]` Test IL gamepad, 2023-09-25: Tested Horipad for Switch, works!
    - `[Y]` Add rotational distinction to wheels, 2023-09-27: Wheels roll convincingly
    - `[Y]` Rotate wheels according to control law, 2023-09-27: Wheels roll convincingly
    - `[>]` Katamari driving
        * `[Y]` Wheels point in stick directions, 2023-09-30: Circled the Square, Fixed some orientation errors
        * `[Y]` Calculate input components from sticks, 2023-09-30: Circled the Square
        * `[>]` Calculate resultant displacement & orientation, One side can drag the other at half speed 
    - `[ ]` Behind & above 3P camera, Allows for seeing ahead and above
    - WARNING: Approaching DANGEROUSLY close to a game engine!
        * `[ ]` Minigame: Cubeling Attack!
            - `[ ]` Cubelings track player and fire explosive pegs
            - `[ ]` Moving player is safe because Cubelings cannot lead their shots
            - `{ }` Obstacles with collision? (DANGER!)
            - `{ }` Threat radar?
        * `[ ]` Actual Dynamics
            - `[ ]` Planar vehicle dynamics with momentum and sliding friction
                * `{ }` Energy-conserving?
            - `[ ]` Non-planar vehicle dynamics
                * `[ ]` Generate uneven terrain (Revive from Synthwave Dream?)
                * `{ }` Jumping?
            
&nbsp;

* `[>]` Animated Bond Graphs
    - `[>]` Render 
        * `[Y]` Major components - 2023-10-10: Render strats for all major components
        * `[Y]` Construct simple example - 2023-10-10: Render strats for all major components
            - `[Y]` Implement Section 3, Slide 13+, One step at a time - 2023-10-10: Render strats for all major components
    - `[>]` Solve 
        * `[>]` Infer causality / flow
            - `[ ]` Render causality 
        * `[ ]` Obtain example solution
    - `[>]` Visualize
        * `[>]` Labeled Nodes
            - `[>]` 1-Junction
            - `[ ]` 0-Junction
        * `[ ]` Flow arrows
            - `[ ]` Animation proportional to flow
        * `[ ]` System var trace emanating from node?
    - `[ ]` Bond Graph vehicle dynamics  
    WARNING: Approaching DANGEROUSLY close to a game engine!
        * `[ ]` Implement paper structs 
        * `[ ]` Implement paper example 
        * `{ }` Animated 3D bond graph? 

&nbsp;

* `[>]` Animated, Alchemal Holographic Star Map (in the style of SW Ahsoka end titles)
    - `[Y]` Revive Icosahedron class, 2023-10-13: Icos and ellipse implemented and drawn
        * `[Y]` Test Icos, 2023-10-13: Icos and ellipse implemented and drawn
    - `[Y]` Revive Sphere class (subdivided icos, MS project), 2023-10-22: Picturesque system map
        * `[Y]` Test Sphere, 2023-10-22: Picturesque system map
    - `[>]` Generate star systems with orbital planes 
        * `[Y]` Generate elliptical orbits, Perturb out of orbital plane, 2023-10-22: Picturesque system map
            - `[Y]` Elliptical torus to represent orbital paths, 2023-10-13: Icos and ellipse implemented and drawn
            - `[Y]` Small chance of highly eccentric, 2023-10-22: Picturesque system map
        * `[ ]` Small chance of retrograde revolution
        * `[Y]` Small chance of retrograde orbit, 2023-10-22: Picturesque system map
        * `[Y]` Small chance of highly obtuse orbital plane, 2023-10-22: Picturesque system map
        * `[Y]` Stars are pictogram billboards, 2023-10-22: Picturesque system map
    - `[ ]` Star-lanes
        * `[ ]` Grow at speed the eye can track without being lethargic
        * `[ ]` Camera tracks
            - `[ ]` Camera selects one to follow, though multiple will grow
    - `[ ]` Shader Example: https://www.youtube.com/watch?v=f4s1h2YETNY
        * `[ ]` Q: How to store state in shaders?
    - `[ ]` Shiny shader
        * `[ ]` Shiny and Handmade, Ancient
            - `[ ]` Generate brush stroke textures
            - `[ ]` Generate normal maps
        * `[ ]` Shade planets
        * `[ ]` Shade orbits
    - `{ }` Runic labels? Source for glyphs?
    - `{ }` Interstellar material? Moving along a flow field?
    


&nbsp;

* `[>]` Engineering Center FROM SPACE - Generative art
    - `[>]` Generative angular, intimidating castle (a la EC), L-system?
        * Classes: `Cuboid` and `Wedge`, 2023-09-06: These are the drawable parts of the EC L-system
        * `[>]` L-system
            - `[Y]` `shared_ptr` Test: Attempt to access child class overridden functions thru a `vector` of `shared_ptr` of the parent class, 2023-09-XX: Requires `virtual`
            - `[>]` Nodes: Drawable, heritable `DynaMesh` 
                * `[Y]` Relative transform, 2023-09-06: These are the drawable parts of the EC L-system
                * `[>]` Ability to rotate about an axis (translation axis)
            - `[Y]` Draw from root to leaves, 2023-09-06: These are the drawable parts of the EC L-system
            - `[ ]` Angular Towers
                * `[ ]` Default drawable spheres for default nodes
                * `[ ]` Test undrawn nodes by optionally drawing them
        * `[ ]` Large Blocks
        * `[ ]` Windows
            - `[ ]` Concrete awnings
        * `[ ]` Bridges
            - `[ ]` "False" loops (Touching but not closed on the tree)
        * `[ ]` Greebles
            - `[ ]` Dishes
            - `[ ]` Cell antennae
            - `{ }` DLC "Wireframe"?
            - `{ }` Scuptures?
    - `[ ]` Textures && Appearance: Only do enough to make it look good!
        * `[ ]` EC
        * `[ ]` Glider
        * `{ }` Bump mapping ???
    - `[ ]` Point Defence - Warning: Do not imply damage to public buildings
        * `[ ]` Friggin' laser beams
        * `{ }` Missiles?
        * `{ }` Primary Weapon ??
            - `{ }` Dynamic lighting for primary weapon?
    - `{ }` Better follow camera: Behind and above
    - `{ }` Gamification - Food delivery ??? : "Eat Fleet", "Food Flotilla"
        * `{ }` Collision handling
            - `{ }` Projectile-Ship
            - `{ }` Ship-EC - Warning: Do not imply damage to public buildings
        * `{ }` Shield
        * `{ }` Chaff / Flares (Limited?)
        * `{ }` Food-delivery Lock-On: Time-delayed "firing solution", Msg: "Delivery Ready"
        * `{ }` Short intro
        * `{ }` Release from carrier
        * `{ }` Win/Lose conditions w/ Game Over screen(s)
    - `[ ]` Video -or- animated GIF output --to-> IG

&nbsp;

* `[P]` Solar Tomography Visualization `[PAUSED]`
    - `[Y]` Parse NPY datacube, 2023-09-13: Read into vector of vector of vector of floats!
    - `[Y]` Visualize raw data as points. Display magnitude with color and opacity, 2023-09-15: Fixed an obvious mistake and data looks sensible now, dots (mini line segments) produce an unappealing moire pattern
        * Empty space has value zero
        * Negative values appear to be artefacts and not real data
    - `[P]` Visualize "cleaned" data as points
        * `[P]` Filter
        * `[ ]` Display magnitude with color and opacity
        * `[ ]` Try cubic or "spherical" voxels to fix the ugly moire of the dots
        * `[ ]` Q: Is there internal structure? If so, what is the best way to render it?
            - `[ ]` Q: What are the names of the structures in this datacube? How are those structures usually depicted?
            - `[ ]` Q: Do plumes have hot "cores"? Can you see through the plumes to their cores?
    - `[ ]` Visualize "cleaned" data as polygons
        * `[ ]` Cluster filtered data
        * `[ ]` Display clusters with Marching Cubes, You have literature on this!
            - `[ ]` Magnitude determines vertex color and opacity
            - `[ ]` E: Think about blending Marching Cubes with dbScan and/or Region Growing Algo such that regions of different magnitude get their own polygons
                * `{ }` Q: When to start a new region in Marching Cubes?
        * `{ }` If marching cubes unsat, then accrue tetrahedra
            - `{ }` Maybe try to grow one tetrahedron at a time?
            - `{ }` Q: Is there a way to add tetrahedra that guarantees the smallest triangles?
            - `{ }` Reject tetrahedra that dilute the points/volume metric --> Start new region
        * `{ }` Depending on findings, consider nested clustering and rendering to match  
            - `{ }` Luminescent cores? -or- Greater luminosity with greater magnitude? 
            - `{ }` Does my method produce/tolerate open polygons? (not airtight)
    - `[ ]` Animated tomography
        * `[ ]` Determine best static display, above
        * `[ ]` Obtain time series of solar tomography data cubes (hypercube?)
        * `[ ]` Animate using best method of static display
        * `[ ]` Q: Are you able to generate in-betweens for greater smoothness?
    - `[ ]` Shaded tomography
        * `[ ]` What is the texture of publicly available sun visualizations?
        * `[ ]` Choose image(s) that evoke the look you are going for
        * `[ ]` Write a shader that implies finer structure/texture than the data cube provides
        * `[ ]` Animate!
            - `[ ]` Try to avoid "jittery" textures

&nbsp;  

* `[ ]` Floating island landscape generation - BoxKart
    - `[ ]` BoxKart
        * `[ ]` Stick to ground
        * `[ ]` "Over the shoulder" 3P camera, similar to flight camera but above player
        * `[ ]` Accounts for bumpy terrain
    - `[ ]` Search for grass shader
    - `[ ]` Boulders and/or mountains
    - `[ ]` Trees as L-systems
    - `[ ]` Water (Simplest)
        * `{ }` Splash effect?
    - `[ ]` Floating islands
        * `{ }` Waterfalls?
    - `{ }` Simple car dyn? Jumps? Falls + respawn?
    - `[ ]` Video -or- animated GIF output --to-> IG

&nbsp;  

* `[ ]` Vaporwave Aesthetic Landscape
    - `[ ]` BoxKart rolling over terrain
        * `[ ]` Vaporwave avatar
    - `[ ]` Fragmented statues
        * `{ }` Iridescent shader?
    - `[ ]` Fragmented architecture
    - `[ ]` Vintage computer window textures
    - `[ ]` Grainy/VHS filter shader

&nbsp;  
&nbsp;  

## Architecture 2: Frames for Fun  (Highly Dangerous!)
* Purpose: Digital sculptures with a dash of simulation
* **Be careful with your time!**

* `[ ]` N-body simulation, orbiting spheres
* `[ ]` L-system tree
* `[ ]` Mesh with animated shader
* `[ ]` L-system generative spacecraft
* `[ ]` Glitch shader
* `[ ]` VHS Shader
* `[ ]` Particle shader

* `[ ]` Destructible Entities
    - `[ ]` Bullet Stress Test, # of Physics Entities at 60 fps
        * `[ ]` Nvidia GPU
        * `[ ]` Intel  NUC
        * `[ ]` Decide on # of allowed physics entities
    - `[ ]` Pre-compute fractures

## Architecture 3: Let's Code (Applied) Physics  (Absurdly Dangerous!)
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Daydream itch

* `[ ]` Investigate compute shaders
    - `[ ]` Decide which is { faster to run, easier to write }
        * `[ ]` OpenCL
        * `[ ]` OpenGL
        * `[ ]` Raylib
    
* `[ ]` Bullet 4x4
    - `[ ]` Terrain class
    - `[ ]` Wheel contact: Clip/Support
    - `[ ]` Wheel friction model
    - `[ ]` Obstacle collision
    
* `[ ]` Find/Create Clifford lib in C++
* `[ ]` Clifford projectile
* `[ ]` Clifford planets
* `[ ]` Clifford electrodynamics
* `[ ]` Electrodynamics
    - `[ ]` Magnets
        * `[ ]` Motor field optimizer
        * `[ ]` Plasma confinement optimizer
    - `[ ]` Generative 

## Architecture N: Shall?  we?  play?  a?  game?
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Have Fun

* `[ ]` RL Agent Class (Is this already supported in Godot?)
    - `[ ]` State exchange format (Graphs? Children hashed?)
    - `[ ]` Slot for brain
    - `[ ]` Advanced Boids Demo

* `[ ]` Planetary Decorator (My Ideal PC Game: Exploration + Casual Architecture + Procedural Generation)
    - `[ ]` Godot tutorials
        * `[ ]` Godot C++ API Tuts
    - `[ ]` Terrain and Landscape generation
    - `[ ]` Choose physics engine: Open Dynamics Engine, Bullet, Godot, etc.
    - `[ ]` Glider with satisfying dynamics and controls
    - `[ ]` Rover with satisfying dynamics and controls
    - `[ ]` Glider <--> Rover transformation
    - `[ ]` Save game w/ persistent landscape
    - `[ ]` Marker system
        * `[ ]` Fast Travel
    - `[ ]` Evaluate Entity Component System (Is this already supported in Godot?)
        * `[ ]` What is the use case of this?
        * `[ ]` Will the capabilities of objects really be that different?
    - `[ ]` Building system
        * `[ ]` Unit blocks
        * `[ ]` Unit walls
        * `[ ]` Parametized handles
        * `[ ]` Parametized cuts
        * `[ ]` Snap
            - `[ ]` pi/6 units to support right angles & hexagons
            - `[ ]` Distance unit snap
            - `[ ]` [Alt] key for grid-free placement
    - `[ ]` Terraforming System
        * `[ ]` Create hills
        * `[ ]` Dig
        * `[ ]` Cut mountains
    - `[ ]` Advanced memory management: Dynamic and Optimized Un/Loading
        * `[ ]` Investigate a loading thread
        * `[ ]` Investigate background loading
        * `[ ]` Subtree/chunk loading decisions like minecraft
        * `[ ]` Decide on a "chunk size"
        * `[ ]` Investigate loading thread w/ queue
        * `[ ]` Garbage collection thread: Can I delay destruction of a shared pointer with no references?
        * `[ ]` Investigate unloading thread w/ queue
    - `[ ]` Graph-based generation
        * `[ ]` What are the aspects that make it believeable?
        * `[ ]` Graph attention 
        * `[ ]` Graph generation
        * `{ }` Evolve params thru user choice?
        * `{ }` Markovian fallback?
    - `[ ]` Game Systems
        * `[ ]` Casual tech tree
            - `[ ]` Mining
            - `[ ]` Geomancy Orbital Ablation Laser
        * `{ }` Autonomous helpers?
            - `{ }` GUI: Agent creation manager
            - `{ }` GUI: Agent priority manager and/or task queue
            - `{ }` Learning Agents & Self-Play
        * `{ }` Wildlife?
        * `{ }` Virtual inhabitants?
    - `[ ]` Story
        * `[ ]` Player character 
            - `[ ]` Model
                * `{ }` Selectable Zook/Ceranid/Mawglin?
            - `[ ]` Rating system for coverage and complexity
        * `[ ]` Terraformer/base mentor character
            - `[ ]` Intro/tutorial sequence
            - `[ ]` Reveal origin/history thru interaction + growing trust
        * `[ ]` Ancient monuments
            [ ]` Choice: Destroy, Collect, Integrate into cities
        * `[ ]` Progression thru planets?
    - `[ ]` Advanced shading
        * `[ ]` Investigate Godot shaders
        * `[ ]` Subdivide drawing mesh for more appealing shading: https://www.cs.ucr.edu/~craigs/courses/2016-fall-cs-130/lab-09.html
        * `[ ]` Evaluate lighted projectiles
        * `[ ]` Investigate shadow casting
        * `[ ]` Investigate ray tracing: Does Godot take advantage of ray tracing hardware?
    - `[ ]` Distribution concerns
        * `[ ]` What platforms will be supported? 
            - `[ ]` What is the Godot pipeline for multi-platform suppport?
            - `[ ]` Steam integration
        * `[ ]` Savegames
        * `[ ]` Screenshots
        * `[ ]` World sharing
        * `[ ]` Co-op play
    - `[ ]` Network play
        * `[ ]` Multiple players
        * `[ ]` Hosted world
* `[ ]` Live Coding IDE

## Completed Projects

* `[Y]` Shiny Boid Ribbons - Generative art, 2023-09-02: Boidular!
    - `[N]` Experiment with memory cycle {Unload Model --> Populate Geo --> Load Mesh --> Load Model}, 2023-08-24: Experiment failed. It appears that the mesh is not actually erased from the GPU?
        * `{Y}` Possibly need flag(s) for this?, 2023-08-24: Flags used.  Did not help!
        * `[N]` If the experiment fails, ask the Discord!, 2023-08-31: Need MVP before asking for help, see "Raylib Lib Rebuild" project
    - `[Y]` `Sphere` class, 2023-09-02: The memory copy issue was due to passing `Sphere`s by value!
    - `[Y]` Boid Ribbon class, 2023-09-02: Boidular!
        * `[Y]` `Basis`, 2023-09-02: Boidular!
        * `[Y]` Logic, 2023-09-02: Boidular!
            - `[Y]` Spherical obstactle avoidance, 2023-09-02: Boidular!
    - `[Y]` Ribbons as dynamic, shaded meshes, 2023-09-02: Boidular!
        * `[Y]` Dynamic mesh, 2023-09-02: Boidular!
        * `[Y]` **Smooth** decay from leading edge, 2023-09-02: Boidular!
        * `[Y]` Add specular glare , 2023-09-02: Boidular!
    - `[Y]` Video -or- animated GIF output --to-> IG, 2023-09-02: Posted!

&nbsp;  

* `[Y]` Boids, 2023-06-24: Satisfying flocking and wandering, 2023-06-24: Satisfying flocking and wandering
    - `[Y]` Boids consider nearby visible neighbors, 2023-06-10: Wrote pose aggregator
    - `[Y]` Instincts driven by gradually-updating transformation matrices, 2023-06-24: Satisfying flocking and wandering
    - `[N]` Add strong obstacle avoidance, 2023-06-24: Too much work for reward
        * `[N]` Investigate Raylib obstacle avoidance, 2023-06-24: Too much work for reward
    - `[Y]` Video -or- animated GIF output --to-> IG, 2023-06-24: Satisfying flocking and wandering
        
&nbsp;  

* `[Y]` Terrain generation - Delta Glider, 2023-06-03: Render plume as a batch job 
    - `[Y]` Perlin Noise terrain grid tiles, 2023-04-12: Tuned scaling and elevation
        * `[Y]` Unshaded with lines, 2023-04-05: Finally got random, non-vibrating triangles w/ correct orientation! Stored in models as tiles instead of individual triangles.
        * `[Y]` Add XY position noise, 2023-04-10: Added
        * `[Y]` Toon shader, 2023-04-12: Works for C++ as well as for C
        * `[Y]` Actual Perlin data, 2023-04-12: Tuned scaling and elevation
            - `{N}` If too smooth, apply random noise to perlin, 2023-04-12: Not needed, looks very natural
    - `[Y]` Synthwave terrain, 2023-04-12: Bloom shader is sufficient
        * `[N]` Write "Synthwave" shader based on toon shader, 2023-04-12: Bloom shader is sufficient
            - `[N]` Display normal shader output to see if you can use this data for the intended shader, 2023-04-12: Bloom shader is sufficient
        * `[Y]` Add bloom glow to boundary lines, 2023-04-12: Bloom shader is sufficient
            - `[Y]` Find bloom shader(s) and implement one example, 2023-04-12: Available in Raylib example shaders, Only slight modification needed
    - `[Y]` JS + KB Ctrl Glider, 2023-04-16: Very smooth
        * `[Y]` Keyboard Ctrl, 2023-04-12: [Arrows] - Pitch & Yaw, [Z/X] - Roll
        * `[Y]` Joystick Ctrl, 2023-04-16: Very smooth
            - `[Y]` RS: Ailerons - Pitch & Roll, 2023-04-16: Very smooth
            - `[Y]` LS: Rudder + Throttle - Yaw & Thrust, 2023-04-16: Very smooth
    - `[Y]` Issue: Massive slowdown and memory usage when there is too much terrain, 2023-04-16: Valgrind is MAGICAL
        * Related?: https://github.com/raysan5/raylib/issues/807, 2023-04-16: Not related
        * `[Y]` Valgrind is MAGICAL, 2023-04-16: Nice!
            - `GenImagePerlinNoise` allocates image on the heap, it needs to be freed with `UnloadImage`
            - Every time that `get_image_pixel_color_at` runs, it copies the ENTIRE IMAGE to a `Color*` which is NOT FREED before the function exits  
    - `[Y]` Infinite grid, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
        * `[Y]` Tiles mesh without terrain discontinuities, even considering XY noise, 2023-04-18: Stitched and blended
        * `[Y]` Add frame rate monitor, 2023-04-17: Added
        * `[Y]` Instantiate without noticeable "pop-in", 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
            - `[Y]` Get Raylib draw distance, 2023-04-22: As below  
            `#define RL_CULL_DISTANCE_NEAR      0.01 // Default projection matrix near cull distance`  
            `#define RL_CULL_DISTANCE_FAR  1000.0 // -- Default projection matrix far cull distance`  
            - `[Y]` Every frame, compute whether to instantiate new neighbors, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
                * `[Y]` Maintain visible terrain up to cull distance, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
                * `[Y]` Keep ahead of cull distance in the direction that the camera is headed, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
            - `[Y]` Every frame, compute whether to show/hide existing tiles, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
            Stop drawing when too far behind, but retain for drawing if player retraces their path  
                * `[Y]` Add and heed a visibility flag, 2023-04-24: Plus `loaded` flag
                * `[Y]` Hide tiles > cull distance behind, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
                * `[Y]` Hide tiles > 2X cull distance ahead, 2023-04-24: Inf terrain without RAM hit, slowdown, or pop-in!
    - `[Y]` Issue: I clearly do not know how shaders work!, 2023-06-03: Vertex colors instead of shaders
        * `[Y]` https://learnopengl.com/Getting-started/Shaders, 2023-05-09: Learned some things!
        * `[Y]` Attempt "Hello Triangle" in Raylib, 2023-06-03: Vertex colors instead of shaders
            - 2023-05-11: OpenGL shader from https://learnopengl.com/Getting-started/Shaders does not seem to run.  Perhaps Raylib shaders are different enough from OpenGL that this would not work. Try [this](https://www.youtube.com/watch?v=8205iyicv5k) instead, 2023-05-13: There is no info at this source
    - `[Y]` Icos Experiments, 2023-06-03: Further experiments not needed
        * `[Y]` Plain icosahedron, 2023-05-01: Icos displayed
        * `[>]` Fading color from bottom to top with a vertex shader
            - 2023-05-02: [Raylib has its own shader language that wraps GLSL](https://github.com/raysan5/raylib/blob/master/src/rlgl.h)
        * `[N]` Dynamic points - Surface position noise?, 2023-06-03: Further experiments not needed
        * `[N]` Dynamic colors?, 2023-06-03: Further experiments not needed
    - `[N]` Distance fog in the form of vanishing lines, 2023-06-03: Cannot be applied to `GL_LINES`
        * https://blog.mapbox.com/drawing-antialiased-lines-with-opengl-8766f34192dc
    - `[y]` Synthwave glider, 2023-06-03: Render plume as a batch job
        * `[Y]` Synthwave shading on glider, 2023-04-12: Bloom shader is sufficient
        * `[Y]` Glowing contrail -or- Wingtip vortices
            - `[Y]` How to fade smoothly?, 2023-06-03: Vertex colors are meant to be accessed
                * Vertex colors?, 2023-04-29: I don't think vertex colors are meant to be accessed, Try a vertex shader  
                * Vertex colors?, 2023-06-03: Vertex colors are meant to be accessed
            - `[Y]` Apply bloom, 2023-06-03: Render plume as a batch job
    - `[N]` Synthwave triangles
        * `[N]` Shootable triangles, Spin
        * `{N}` Iridescent/glare?, 2023-04-12: Would distract from vector aesthetic
    - `{N}` Curved CRT shader?, 2023-06-03: Render plume as a batch job
    - `[Y]` Video -or- animated GIF output --to-> IG, 2023-06-03: Render plume as a batch job 
        - `[N]` Remove CRT curve for IG post, 2023-06-03: Render plume as a batch job

&nbsp;  

```
[Y] Grid Glider - Delta Glider
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
    [Y] Video -or- animated GIF output --to-> IG, 2022-12-2X: Shared 
```

# Resources


# nanoverse
Artistic projects rendered in various frameworks and engines, written in (and grouped by) various languages 
* Keep work on this toy to a responsible minimum
* All units in meters `[m]`


## Architecture 1: C++ & RayLib  (Dangerous!)
* Purpose: Let yourself have some graphics, as a treat!  

* `[>]` Terrain generation - Delta Glider

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

    - `[>]` Synthwave glider
        * `[Y]` Synthwave shading on glider, 2023-04-12: Bloom shader is sufficient
        * `[>]` Glowing contrail -or- Wingtip vortices
            - `[>]` How to fade smoothly?
                * Vertex colors?

    - `[ ]` Synthwave triangles
        * `[ ]` Collision triangles, Spin
        * `{N}` Iridescent/glare?, 2023-04-12: Would distract from vector aesthetic
    - `[ ]` Infinite grid
        * `[ ]` Tiles mesh without terrain discontinuities, even considering XY noise
        * `[ ]` Distance fog
        * `[ ]` Add frame rate monitor
        * `[ ]` Instantiate without noticeable "pop-in"
        * `[ ]` Erase when too far behind
    - `{ }` Curved CRT shader?
    - `[ ]` Video -or- animated GIF output --to-> IG 
        - `[ ]` Remove CRT curve for IG post

* `[>]` Piped Ribbons - Generative art
    - `[Y]` Randomly trace a 3D grid, with 90deg curved turn limit with radius that lands mid-segment
    - `[Y]` Ribbons as contrails, 2022-12-2X: Shared 
        * `[Y]` Deterministic turning rules that allow predictable loop, 2022-12-2X: Shared 
        * `[Y]` Multiple, random non-colliding ribbons, 2022-12-2X: Shared 
    - `[ ]` Replace loop w/ simple path planning that avoids collisions and has a center-seeking tendency
    - `[ ]` Ribbons as dynamic, shaded meshes
        * `[ ]` Dynamic mesh
        * `[ ]` **Smooth** decay from leading edge
        * `[ ]` Add specular glare 
        * `[ ]` Bloom shader
    - `[ ]` Video -or- animated GIF output --to-> IG

* `[ ]` Engineering Center FROM SPACE - Generative art
    - `[ ]` Generative angular, intimidating castle (a la EC), L-system?
        * `[ ]` Windows
        * `[ ]` Blocks
            - `[ ]` Raised blocks
        * `[ ]` Bridges
        * `[ ]` Greebles
            - `[ ]` Dishes
            - `[ ]` Cell antennae
    - `[ ]` Textures
    - `[ ]` Rotating "turrets"
        * `[ ]` Friggin' laser beams
        * `{ }` Dynamic lighting for primary weapon?
    - `{ }` Fends off Delta Gliders? - Warning: Do not imply damage to public buildings
        * `{ }` Playable?  Food delivery?
    - `[ ]` Video -or- animated GIF output --to-> IG


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


* `[ ]` Vaporwave Aesthetic Landscape
    - `[ ]` BoxKart rolling over terrain
        * `[ ]` Vaporwave avatar
    - `[ ]` Fragmented statues
        * `{ }` Iridescent shader?
    - `[ ]` Fragmented architecture
    - `[ ]` Vintage computer window textures
    - `[ ]` Grainy/VHS filter shader



## Architecture 2: Frames for Fun  (Highly Dangerous!)
* Purpose: Digital sculpture platform with a dash of simulation
* Be careful with your time

* `[ ]` N-body simulation, orbiting spheres
* `[ ]` L-system tree
* `[ ]` Mesh with animated shader
* `[ ]` Load ship models into space scene
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



## Architecture 4: Advanced Simulation
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Play with a 3D game engine (Godot)
* Fast & Parallel

* `[ ]` Clifford electrodynamics
* `[ ]` Electrodynamics
    - `[ ]` Magnets
        * `[ ]` Motor field optimizer
        * `[ ]` Plasma confinement optimizer
    - `[ ]` Generative 

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


    



## Architecture 5: A Sim for All Seasons
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Share what you made

* `[ ]` FINCH Integration
* `[ ]` Network play
    - `[ ]` Multiple players
    - `[ ]` Hosted world
    - `[ ]` Agent exchange
* `[ ]` Live Coding IDE



## Architecture N: Shall?  we?  play?  a?  game?
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Have Fun

* `[ ]` RL Agent Class (Is this already supported in Godot?)
    - `[ ]` State exchange format (Graphs? Children hashed?)
    - `[ ]` Slot for brain
    - `[ ]` Advanced Boids Demo


## Completed
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


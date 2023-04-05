# nanoverse
Artistic projects rendered in various frameworks and engines, written in (and grouped by) various languages 
* Keep work on this toy to a responsible minimum
* All units in meters `[m]`


## Architecture 1: C++ & RayLib (Dangerous!)
* Purpose: Let yourself have some graphics, as a treat!  

```

[>] Terrain generation - Delta Glider
    [>] Perlin Noise grid hills
        [Y] Unshaded with lines, 2023-04-05: Finally got random, non-vibrating triangles w/ correct orientation!
        [>] Toon shader
        [ ] Actual Perlin data
    [ ] Synthwave terrain
        [ ] Write "Synthwave" shader based on toon shader
        [ ] Add bloom glow to boundary lines
    [ ] JS Ctrl Glider
        [ ] Joystick test
        [ ] Synthwave shading
    [ ] Synthwave triangles
        [ ] Collision triangles, Spin
    [ ] Infinite grid
    [ ] Video -or- animated GIF output --to-> IG


[ ] Landscape generation - Delta Glider
    [ ] Search for grass shader
    [ ] Boulders and/or mountains
    [ ] Trees as L-systems
    [ ] Grass (as shader?)
    [ ] Water (Simplest)
        [ ] Whoosh effect?
    [ ] Floating islands
        [ ] Waterfalls?
    [ ] Video -or- animated GIF output --to-> IG

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

[>] Braitenberg Vehicles
    [>] Test message passing system
    [ ] Implement components for the first vehicle
    [ ] Implement the first vehicle
        [ ] Test movement in text
    [ ] Add graphical representation for the components so far
    [ ] Graphical representation of an entire vehicle
    [ ] Vehicle moves through the environment

[>] Piped Ribbons - Generative art
    [Y] Randomly trace a 3D grid, with 90deg curved turn limit with radius that lands mid-segment
    [Y] Ribbons as contrails, 2022-12-2X: Shared 
        [Y] Deterministic turning rules that allow predictable loop, 2022-12-2X: Shared 
        [Y] Multiple, random non-colliding ribbons, 2022-12-2X: Shared 
    [>] Ribbons as shaded meshes
        [ ] Test simple shaders && Learn about them
        [ ] Add simple shader
        [ ] Add luminocity to shader
        [ ] Add decay from leading edge to shader
    [ ] Video -or- animated GIF output --to-> IG

 

[ ] Tetrahedra - Generative Art
    [ ] ???

```

## Architecture 2: Frames for Fun
* Purpose: Digital sculpture platform with a dash of simulation
* Be careful with your time
```
[ ] N-body simulation, orbiting spheres
[ ] L-system tree
[ ] Mesh with animated shader
[ ] Load ship models into space scene
[ ] Glitch shader
[ ] VHS Shader
[ ] Particle shader

```

## Architecture 3: Let's Code (Applied) Physics
* Purpose: Daydream itch
```
[ ] Find/Create Clifford lib in Dlang
[ ] Clifford projectile
[ ] Clifford planets
[ ] Clifford electrodynamics
[ ] Electrodynamics
    [ ] Magnets
        [ ] Motor field optimizer
        [ ] Plasma confinement optimizer
    [ ] Generative 

```

## Architecture 4: Advanced Simulation
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Play with a 3D game engine (Godot)
* Fast & Parallel
```
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

```

## Architecture 5: A Sim for All Seasons
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Share what you made
```
[ ] FINCH Integration
[ ] Network play
    [ ] Multiple players
    [ ] Hosted world
    [ ] Agent exchange
[ ] Live Coding IDE
```


## Architecture N: Shall?  we?  play?  a?  game?
### DO NOT ATTEMPT WITHOUT AN ESTABLISHED CAREER!
* Purpose: Have Fun
```
[ ] Evaluate Modular Refactor:
    [ ] Consider Interfaces for each kind of activity: { Graphics, Control, Intelligence, More? }
    [ ] Does it make sense to have an interface for each?
    [ ] Does an interface imply a plugin interface?
    [ ] If using plugins, Connect at interface in a way that reduces pointer math and communication overhead
        For example, Using multiple interfaces handled with an inheritance list 
        that facilitates information flow through pointers on both sides such that s/getters are never used?
        https://www.arbinada.com/en/node/1466
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


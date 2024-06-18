# `DEV PLAN`


## `[>]` Computer Graphics Blitz in Pure C + OpenGL + GLUT, Morning Warmup and Weekend Project
* Goal: Reinforce fluency in Pure C + Pure OpenGL. I have been spoiled by Raylib + C++!
* C++, Eigen, GLFW, and GLM are **DISALLOWED**!


### HW04+05: Third-Person View in a Scene + Phong Shading + Moving Light Source(s) + Particle System(s)
* `[Y]` TetraTank rolling on icos wheels, 2024-05-1X: MOUSELOOK!
    - `[Y]` mouselook, 2024-05-1X: MOUSELOOK!
    - `{N}` Joystick control, 2024-05-20: GLUT lacks dual-stick gamepad handling! Mouselook is sufficient
    - `{Y}` Hide cursor while in the window?, 2024-05-1X: Hidden!
    - `{N}` Wireframe as VBO?, 2024-06-18: This is not interesting
        * [OpenGL Wireframe Mode](https://stackoverflow.com/a/137649)
* `[>]` Fireworks
    - `[Y]` Fire shell from tank barrel, 2024-05-2X: Fires!
    - `[>]` Shell has a particle spark trail
        * `[>]` Implement as a geometry shader writing to a texture buffer
        * `[ ]` Second geometry shader turns points into billboards
    - `[ ]` Shell emits a strong light
    - `[ ]` Firework Detonation
        * `[ ]` Shell ends trajectory with a Poisson distribution
        * `[ ]` To start, use a simple mesh as streaks (fading opacity)
        * `[ ]` Use the star texture from the [geometry shader example](https://www.prinmath.com/csci5229/Sp24/programs/ex18.zip)
        * `[ ]` Emit **very** strong light
        * `[ ]` Generate shockwave of particle sparks
        * `[ ]` Sparks drop and fade
* `[>]` Environment
    - `[P]` Moon (Stolen from [Vlakkies' Solar System demo](https://www.prinmath.com/csci5229/F23/programs/ex19.zip))
    - `{P}` Stars?
    - `[Y]` Planar terrain from subdivided triangles, 2024-05-2X: Completed, even shading
    - `[ ]` Ambient + Directional moonlight
        * `[ ]` Terrain matte
        * `[ ]` Tank glossy

    
### HW06: Shaded & Textured Models
* `[>]` Texture the Chopperhedron
    - `[ ]` Test
* `[ ]` Randomized EC towers w/ texture
    - `[ ]` Choose EC "Unit Square"
    - `[ ]` Prep textures: {Select, Squarify, Downsample}
* `[>]` Chopperhedron Animation Scripts
    - `[>]` _Cheap_ Behavior Treeps ( _CAREFUL!_ )
    - `[ ]` Simplest keypoint animation via BT
    - `[ ]` Running modes
    - `[ ]` Transformation Sequence
* `{?}` Use a shader to construct lines at edges
    - [Computing Barycentric Coordinates](https://tchayen.github.io/posts/wireframes-with-barycentric-coordinates)
    - `{?}` GLSL: How to compute distance from a triangle edge?  Sounds like a geometry shader to me?
* `{?}` Chopper mode? ( _DANGER!_ )
    - `{?}` SDL2 gamepad integration
    - `{?}` Floaty Dynamics
    

### HW07: Display Lists
### HW08: Aliased Lines
### HW09: Bezier Curves
### HW10: Bezier Surfaces
* `[ ]` Look at: Utah Teapot Example
### HW11: B-Splines
### HW12: B-Spline Surfaces
* `[ ]` Smoothly undulating terrain
* `[ ]` TetraTank wheels track terrain
### HW13: Engineering Center as a Procedural L-System

### Example Project: Shadow Mapping
* `[ ]` Directional shadows from moonlight
* `[ ]` Shadows cast on player character
    - `[ ]` Tron highlights still shine when in shadow
* `{?}` Gameplay ( _EXTREME DANGER!_ )
    - `{?}` Enemies: Floating Eyes
        * `{?}` Detect visible/obscured?
    - `{?}` **BASIC** collistion detection
    - `{?}` Player HUD
    - `{?}` SDL2 Multimedia
        * `{?}` Sound effects
        * `{?}` BG music


### Example Project: Physical-Based Rendering in OpenGL
* `[ ]` Find examples
* `[ ]` Re-read email from Paradise Killer programmer
* `[ ]` Crystalline models with subsurface flakes
* `[ ]` Crystalline models in EC scene + lighting


## `[ ]` Advanced Computer Graphics + Geometric Algebra ( _YAGNI!_ )
* C: -O3 -Wall -std=gnu17
* OpenGL Visualization
### `[ ]` Book --to-> Library
* `[ ]` Decide on a Reading Plan
* `[ ]` Create a Development Plan
### `[ ]` N Body Problem
* `[ ]` Search for interesting orbits to render
### `[ ]` Classical Forces & Torques ( _DANGER!_ )
### `[ ]` Classical Electrodynamics ( _EXTREME DANGER!_ )
### `[ ]` Relativistic Systems ( _EXISTENTIAL DANGER!_ )


# Completed Projects

## `[>]` Computer Graphics Blitz in Pure C + OpenGL + GLUT, Morning Warmup and Weekend Project
* Goal: Reinforce fluency in Pure C + Pure OpenGL. I have been spoiled by Raylib + C++!
* C++, Eigen, GLFW, and GLM are **DISALLOWED**!


### `[Y]` HW03: Nested Reference Frames with `glPushMatrix`/`glPopMatrix`, COMPLETE
* 2024-05-21: Needs testing, To combine with the "HW4" project, below
* `[Y]` Simple VBO struct with vertex coloring, 2024-05-12: Needs testing
* `[Y]` Render a cube, 2024-05-12: Tested!
* `[Y]` Render rotating cubes in nested reference frames, 2024-05-16: Nested rotating cubes!
    - `[Y]` Single rotating cubes, 2024-05-16: Nested rotating cubes!
    - `[Y]` Multiple levels of rotating cubes, 2024-05-16: Nested rotating cubes!
* `[Y]` Promote "02_VBO_VNC_f.c" to a library  &&  Add VBO shapes, 2024-05-17: Added!
    - `[Y]` Lib, 2024-05-16: Promoted!
    - `[Y]` Add VBO Icos, 2024-05-17: Added!
    - `[Y]` Add VBO Tetra, 2024-05-17: Added!
* `[Y]` X-Y Grid, 2024-05-17: Ported from coursework, Needs testing
    - `{P}` As VBO?, 2024-05-17: Do this later.  Add one thing at a time!
* `[Y]` TetraTank rolling on icos wheels, 2024-05-21: Needs testing
    - `[Y]` kb movement, 2024-05-1X: Move with either arrows or WASD
    - `[Y]` Convincing rolling contact, 2024-05-21: Needs testing

### `[Y]` HW02: Lorenz Attractor, COMPLETE
* `[Y]` Multiple, Chasing Traces of System State, 2024-05-0X: 6D Lorenz attractor completed and tuned!
    - `[Y]` Different starting state, Different eq. params, 2024-05-0X: 6D Lorenz attractor completed and tuned!
    - `{Y}` Are there versions of the attractor with higher-dimensional state?, 2024-05-0X: 6D Lorenz attractor completed and tuned!

## Other Demos

### `[Y]` Planet Atmosphere + Compute Shader, COMPLETE
* 2024-05-12: Done!
* `[Y]` Orthogonal  Cuboid, 2024-04-22: HELL YES
* `[Y]` Perspective Cuboid, 2024-04-22: FUCK YES
* `[Y]` Sphere, 2024-04-22: LET'S FUCKING GOOOOO
* `[Y]` Geo Shader Tut, 2024-04-22: Geo shader does not really store non-geo state, Not executed in particular order
    - `[N]` Choose a tut to implement, 2024-04-22: May not be the correct tool for the job
* `[Y]` Left-to-Right Argument convention!, 2024-04-27: Implemented a consistent convention for naming functions and ordering function arguments that reflects C's Left-to-Right lexical flow of data for assignment.  Conventions should make thinking about the problem **easier**!
* `[Y]` Atmos particles on the CPU, 2024-04-29: Close enough!, As long as there are guards against zero accel then the simulation does not COLLAPSE. <-- Be careful of this while refactoring for the GPU
    - `[Y]` Icos, 2024-04-23: Drawn
        * `[Y]` Draw as wireframe, 2024-04-23: Drawn
    - `[Y]` Icos net, 2024-04-27: Issue solved!
        * `[N]` Port neighbor-finding code from MS, 2024-04-27: That code was naturally overly complex
        * `[Y]` ISSUE: NOT all neighbors are found!, 2024-04-27: There was an issue with the uint row loader!
            - `[Y]` Test with a simple tetrahedron, 2024-04-27: PASSED!
    - `[Y]` Per-face coordinate system, 2024-04-27: Testing req'd
    - `[Y]` Per-face particles, 2024-04-27: They fly!
        * `[Y]` Init particles, 2024-04-27: Testing req'd
        * `[Y]` Render: Particles + Wireframe + Planet, 2024-04-27: They fly!
            - `[Y]` Project particles to 3D, 2024-04-27: Testing req'd
    - `[Y]` Per-face particle motion w/ appealing particle dynamics, 2024-04-27: They fly!
        * `[Y]` Each cell imparts a fixed accel per frame, 2024-04-27: They fly!
        * `[Y]` Clip particle vel at some appropriate value, 2024-04-27: They fly!
        * `[Y]` Animate, ignore cell bounds, 2024-04-27: They fly!
    - `[Y]` Track boundary-crossing, 2024-04-28: Working! Some triangles still get starved
        * `[Y]` Membership test: Detect particle exit, 2024-04-28: Testing req'd
        * `[Y]` Particle transfer, 2024-04-28: Working! Opposing triangle cause wonky fronts.
            - `[Y]` Project R^3 position to R^2 for new cell, 2024-04-28: Working! Opposing triangle cause wonky fronts.
            - `[Y]` Remember to wrap insertion indices --> If over limit, Then new particle writes over interior particle, 2024-04-28: Working! Opposing triangle cause wonky fronts.  
            - `[Y]` Track lost particles, 2024-04-28: Working! Opposing triangle cause wonky fronts.
        * `[Y]` Backfill lost particles, 2024-04-28: Working! Some triangles still get starved
    - `[Y]` Cosmetic improvements, 2024-04-30: It's fucking **GPU TIME**
        * `[Y]` Flip non-convex triangles *during* icosphere construction, 2024-04-29: Net was properly constructed!
        * `[Y]` {Diffusion, Perturbation} phase for natural current adjustment, 2024-04-29: Added a warm-up period to init
            - `[Y]` 20+ cycles at init to avoid wonky fronts, 2024-04-29: Added a warm-up period to init
            - `[Y]` 60+ frames at init to avoid triangular clouds, 2024-04-29: Added a warm-up period to init
        * `[Y]` Particle Generation
            - `[Y]` Particle gen function, Apply some light clumping to encourage streams, 2024-04-29: After about a second of running time this starts to produce appealing streams. Added a warm-up period to init
            - `[Y]` Account for net {Entry,Exit} in `lost` (in addition to overwrites), 2024-04-30: Cell starvation **averted**!
            - `[Y]` Only allow a *fraction* of `lost` to be replaced each frame to prevent "pop-in" (Rate of addition exponentially decays), 2024-04-30: 1/96 per frame seems to prevent overcrouding
        * `[Y]` Move specific nets {icos,icosphere,tetra} to "TriNet.h", 2024-04-30: Moved for safe keeping
* `[Y]` Prep for GPU
    - `[Y]` DELETE CLASS ASSIGNMENT CODE, 2024-05-01: Done
    - `[Y]` Move init params to program globals, there is only one simulation running!, 2024-05-01: This is already part of the example
    - `[Y]` Compute shader test, Simplest particle motion: See adv'd graphics Example 20 @ prinmath.com, 2024-05-01: It works!
        * https://www.prinmath.com/csci5229/Sp24/programs/index.html
        * `[Y]` Copy minimal code from Example, 2024-05-01: Stripped out GLFW
        * `[Y]` Remove sphere, 2024-05-01: Sure why not
        * `[N]` Change particle dynamics, 2024-05-01: Can't be bothered
* `[P]` Atmos particles on the GPU
     - Concepts
        * *DE*-compartmentalize!, Prefer **arrays** over structs: **NO** more particle exchange *between* structs!
        * Prefer lookups to copying
            - Particles are *not* confined to their cell
            - *NO* backill operation
    - Ideas:
        * Kill particles that reach zero velocity?, WARNING: REQUIRES BACKFILL
    - `[>]` Program Reorg
        * `[>]` Copy minimal code from Modified Example
        * `[>]` **ALL** tuning params as **GLOBALS**
        * `[>]` **ALL** particle data as *FAT* arrays
    - `[Y]` Draw sphere, Test
    - `[Y]` Init particles
        * `[Y]` Choose cell randomly, Choose size randomly
        * `[Y]` Locate clump centroid
        * `[Y]` Generate clump as Gaussian
    - `[Y]` GPU Data: Init Atmosphere
        * `[Y]` One Row per Particle
            - `[Y]` Position, 2024-05-01: Allocated on GPU
            - `[Y]` Velocity, 2024-05-01: Allocated on GPU
            - `[Y]` Color, 2024-05-01: Allocated on GPU
            - `[Y]` Membership Index, `uint`, 2024-05-01: Allocated on GPU
        * `[Y]` One Row per Cell, 2024-05-01: Allocated on GPU
            - `[Y]` Origin, 2024-05-01: Allocated on GPU
            - `[Y]` Vertex 1, 2024-05-01: Allocated on GPU
            - `[Y]` Vertex 2, 2024-05-01: Allocated on GPU
            - `[Y]` X Basis, 2024-05-01: Allocated on GPU
            - `[Y]` Y Basis, 2024-05-01: Allocated on GPU
            - `[Y]` Acceleration, 2024-05-01: Allocated on GPU
            - `[Y]` Neighbors, 2024-05-01: Allocated on GPU
    - `[Y]` Draw particles, 2024-05-12: Drawn!
    - `[Y]` GPU: Compute Shader, One worker per *particle* (Easiest adaptation), Dispatch for dynamics:, 2024-05-12: Drawn!
        * `[Y]` Calc mass from color, Scale accel by `(r+g+b)/3.0f` (Most intense particles are fastest), 2024-05-12: Drawn!
        * `[Y]` Membership lookup, 2024-05-12: Drawn!
        * `[Y]` Advance particle in 3D, 2024-05-12: Drawn!
    - `[Y]` GPU: Compute Shader, One worker per *particle* (Easiest adaptation), Dispatch for dynamics:, 2024-05-12: Drawn!
        * `[Y]` Departure check in 3D, 2024-05-12: Drawn!
        * `[Y]` If departed, then update membership and project to new location, 2024-05-12: Drawn!
        * `{N}` Consider: One worker per *section*, 2024-05-12: No, just move on ...
    - `[Y]` CPU: Wind Dynamics, Treat speed and direction *separately*, 2024-05-12: Drawn!
        * `[Y]` Only ONE diffusion exchange per 4-neighborhood, Center modified ONCE, 2024-05-12: Drawn!
            - `[Y]` Blend direction, 2024-05-12: Drawn!
            - `[Y]` Blend speed, 2024-05-12: Drawn!
        * `[Y]` Perturbation, 2024-05-12: Drawn!
            - `[Y]` Nudge direction, 2024-05-12: Drawn!
            - `[Y]` Nudge speed, 2024-05-12: Drawn!
        * `[Y]` Enforce Nonzero Speed, 2024-05-12: Drawn!
            - `[Y]` If diffusion cancels, Randomize, 2024-05-12: Drawn!
        * `[Y]` Write wind direction array back to GPU, 2024-05-12: Drawn!
    - `[Y]` Enable lighting, 2024-05-12: Drawn!
    - `[Y]` Share, 2024-05-12: Done!
        * `[Y]` Record video, 2024-05-12: Compression has NO IDEA what to do with 1.5M particles, lol!
        * `[Y]` Post insta, 2024-05-12: Compression has NO IDEA what to do with 1.5M particles, lol!
* `[Y]` Post-Mortem, 2024-05-12: Done!
    - BUG: I *never* did a change of basis while blending accelerations in the CPU version!
    - BUG: In the CPU version, if a cell ever has zero acceleration, then ALL PARTICLES DISAPPEAR!
    - `[Y]` Consider: Use the example `vec4f` union struct, 2024-05-12: Done!
        * `|+|` C functions can return structs
        * `|+|` C functions can pass structs by value
        * `|+|` Don't pay for dereference overhead
        * `|+|` Easier to read, None of this --> `\*<<*\`
        * `|-|` Pass by value uses stack space
    - `[Y]` Re-write utils as "OGL_Tools.h", considering above , 2024-05-12: Done!
    - `[Y]` Re-write "TriNet.h", considering above , 2024-05-12: Done!

* `[ ]` Q: Is it better for a worker to work on a single point or a smallish collection of points?
* `[ ]` Q: How to add shading to the atmosphere to match the planet?

# Far Future
See also: Top-level plan
## Raylib Improvements
* `[ ]` Add computer shaders
    - `[ ]` Ask Ray **BEFORE** *starting*!
    - `[ ]` Pull Request!
# `DEV PLAN`

## `[>]` Planet Atmosphere + Compute Shader 

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
    - `[ ]` Draw sphere, Test
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
            
    - `[>]` Draw particles

    - `[ ]` GPU: Compute Shader, One worker per *particle* (Easiest adaptation), Dispatch for dynamics:
        * `[ ]` Calc mass from color, Scale accel by `(r+g+b)/3.0f` (Most intense particles are fastest)
        * `[ ]` Membership lookup
        * `[ ]` Advance particle in 3D

    - `[ ]` GPU: Compute Shader, One worker per *particle* (Easiest adaptation), Dispatch for dynamics:
        * `[ ]` Departure check in 3D
        * `[ ]` If departed, then update membership and project to new location
        * `{?}` Consider: One worker per *section*

    - `[ ]` CPU: Wind Dynamics, Treat speed and direction *separately*
        * `[ ]` Only ONE diffusion exchange per 4-neighborhood, Center modified ONCE
            - `[ ]` Blend direction
            - `[ ]` Blend speed
        * `[ ]` Perturbation
            - `[ ]` Nudge direction
            - `[ ]` Nudge speed
        * `[ ]` Enforce Nonzero Speed
            - `[ ]` If diffusion cancels, Randomize
        * `[ ]` Write wind direction array back to GPU
    - `[ ]` Share
        * `[ ]` Record video
        * `[ ]` Post insta
        * `[ ]` Q: Is it better for a worker to work on a single point or a smallish collection of points?
        * `[ ]` Q: The advanced class example 20 seems to run less smoothly on maximum settings on my home machine.  Is it because I replaced GLFW with GLUT?

* `[P]` Post-Mortem
    - BUG: I *never* did a change of basis while blending accelerations in the CPU version!
    - BUG: In the CPU version, if a cell ever has zero acceleration, then ALL PARTICLES DISAPPEAR!
    - `[ ]` Consider: Use the example `vec4` union struct
        * `|+|` C functions can return structs
        * `|+|` C functions can pass structs by value
        * `|+|` Don't pay for dereference overhead
        * `|+|` Easier to read, None of this --> `\*<<*\`
        * `|-|` Pass by value uses stack space
    - `[ ]` Re-write utils as "OGL_Tools.h", considering above 
    - `[ ]` Re-write "TriNet.h", considering above 


## `[ ]` Computer Graphics Blitz in Pure C + OpenGL + GLUT, Morning Warmup and Weekend Project
* Goal: Reinforce fluency in Pure C + Pure OpenGL. I have been spoiled by Raylib + C++!
* C++ and Eigen are **DISALLOWED**!
* Compile: `gcc -std=gnu17 -O3 -Wall <SOURCE FILE>.c -lglut -lGLU -lGL -lm -o <PROG NAME>.out`

### HW2: Lorenz Attractor
* `[ ]` Multiple, Chasing Traces of System State
    - `[ ]` Different starting state, Different eq. params
    - `{?}` Are there versions of the attractor with higher-dimensional state?

### HW3: Nested References Frames with `glPushMatrix`/`glPopMatrix`
* `[ ]` View newer class material

### HW4: First-Person View in a Scene
* `[ ]` View newer class material

### HW5: Phong Shading + Moving Light Source(s)
* `[ ]` View newer class material

### HW6: Shaded & Textured Models
* `[ ]` View newer class material

### HW7: ???
* `[ ]` View newer class material

### Example Project: Shadow Mapping
* `[ ]` View newer class material
* `{?}` Joystick interaction thru GLUT?
* `{?}` Detect visible/obscured?


## `[ ]` Advanced Computer Graphics in Pure C + OpenGL + GLUT
* C++ and Eigen are **DISALLOWED**!

### HW2: Lorenz Attractor
* `[ ]` Geometry shader to expand line into a ribbon

### ???
* `[ ]` ??????

# Far Future
See also: Top-level plan
## Raylib Improvements
* `[ ]` Add computer shaders
    - `[ ]` Ask Ray **BEFORE** *starting*!
    - `[ ]` Pull Request!
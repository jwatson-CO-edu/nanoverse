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

* `[>]` Prep for GPU
    - `[Y]` DELETE CLASS ASSIGNMENT CODE, 2024-05-01: Done
    - `[Y]` Move init params to program globals, there is only one simulation running!, 2024-05-01: This is already part of the example
    - `[>]` Compute shader test, Simplest particle motion: See adv'd graphics Example 20 @ prinmath.com
        * https://www.prinmath.com/csci5229/Sp24/programs/index.html
        * `[>]` Copy minimal code from Example
        * `[>]` Remove sphere
        * `[>]` Change particle dynamics

* `[P]` Atmos particles on the GPU
     - Concepts
        * *DE*-compartmentalize!: **NO** more particle *exchange*
        * Prefer lookups to copying
            - Particles are *not* confined to their cell
            - *NO* backill operation
    - Ideas:
        * Kill particles that reach zero velocity?
    - `[>]` Any questions?
        * `[ ]` Q: Is it better for a worker to work on a single point or a smallish collection of points?
    - `[Y]` Ask V: How to store persistent state on the GPU?, 2024-04-27: See adv'd graphics Example 20 @ prinmath.com
    - `[Y]` Ask V: Compute Shader --to-> Geometry Shader?, 2024-04-27: See adv'd graphics Example 20 @ prinmath.com
    - `[ ]` Program Reorg
        * `[ ]` **ALL** tuning params as **GLOBALS**
        * `[ ]` **ALL** particle data as *FAT* arrays
    - `[ ]` Compute Shader
        * `[ ]` One worker per *particle* (Easiest adaptation)
            - `[ ]` Dispatch for dynamics
                * `[ ]` Membership lookup
                * `[ ]` Advance particle in 3D
                * `[ ]` Departure check in 3D
        * `{?}` Consider: One worker per *section*




## `[ ]` Computer Graphics Blitz in Pure C + OpenGL + GLUT
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
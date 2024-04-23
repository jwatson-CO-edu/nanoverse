# `DEV PLAN`
* `[Y]` Orthogonal  Cuboid, 2024-04-22: HELL YES
* `[Y]` Perspective Cuboid, 2024-04-22: FUCK YES
* `[Y]` Sphere, 2024-04-22: LET'S FUCKING GOOOOO
* `[Y]` Geo Shader Tut, 2024-04-22: Geo shader does not really store non-geo state, Not executed in particular order
    - `[N]` Choose a tut to implement, 2024-04-22: May not be the correct tool for the job
* `[>]` Atmos particles on the CPU
    - `[Y]` Icos, 2024-04-22: Drawn
        * `[Y]` Draw as wireframe, 2024-04-22: Drawn
    - `[>]` Icos net
        * `[>]` Port neighbor-finding code from MS
    - `[ ]` Per-face coordinates (code from MS)
    - `[ ]` Per-face particles
        * `[ ]` Particles + Wireframe
    - `[ ]` Per-face particle motion
        * `[ ]` Animate, ignore cell bounds
    - `[ ]` Track boundary-crossing
        * `[ ]` Detect particle exit
        * `[ ]` Project R^3 position to R^2 for new cell
        * `[ ]` Limit 32 particles
        * `[ ]` If over limit, Then new particle writes over interior particle
* `[ ]` Atmos particles on the GPU
    - `[ ]` Ask V: How to store persistent state on the GPU?
    - `[ ]` Ask V: Compute Shader --to-> Geometry Shader?
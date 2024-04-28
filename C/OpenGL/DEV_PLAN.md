# `DEV PLAN`
* `[Y]` Orthogonal  Cuboid, 2024-04-22: HELL YES
* `[Y]` Perspective Cuboid, 2024-04-22: FUCK YES
* `[Y]` Sphere, 2024-04-22: LET'S FUCKING GOOOOO
* `[Y]` Geo Shader Tut, 2024-04-22: Geo shader does not really store non-geo state, Not executed in particular order
    - `[N]` Choose a tut to implement, 2024-04-22: May not be the correct tool for the job
* `[Y]` Left-to-Right Argument convention!, 2024-04-27: Implemented a consistent convention for naming functions and ordering function arguments that reflects C's Left-to-Right lexical flow of data for assignment.  Conventions should make thinking about the problem **easier**!
* `[>]` Atmos particles on the CPU
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

    - `[>]` Track boundary-crossing
        * `[>]` Membership test: Detect particle exit
        * `[ ]` Project R^3 position to R^2 for new cell
        * `[ ]` Remember to wrap insertion indices --> If over limit, Then new particle writes over interior particle  

* `[ ]` Compute shader test
    - `[ ]` ???
* `[P]` Atmos particles on the GPU
    - `[Y]` Ask V: How to store persistent state on the GPU?, 2024-04-27: See adv'd graphics Example 20 @ prinmath.com
    - `[Y]` Ask V: Compute Shader --to-> Geometry Shader?, 2024-04-27: See adv'd graphics Example 20 @ prinmath.com
    - `[ ]` ???
# Bond Graph Synthesis
1. Create a system FBD
1. Identify and label all distinct, non‐zero, absolute flows / velocities in the physical model
    * Velocities of masses
    * Velocities at the ends of springs and dampers
    * Indicate the arbitrarily‐assumed positive velocity directions, Relative to an inertial reference
1. Choose the relative velocities of the springs and dampers to be positive either in compression or in tension
1. Note gravitational forces
1. List all one‐ and two‐port elements along with their relevant flows / velocities
    * Map physical components to one‐port bond graph components
    * Include component values
    * Define relative velocities as differences between absolute velocities
    * Sources move with the components to which they’re connected
    * Include bonds connected to one‐ports
        - Direction of source bonds determined by power convention
        - Bonds point in toward all I’s, R’s, and C’s
1. Place a 1‐junction for each distinct flow / velocity
    * Both absolute and relative velocities
    * Label the velocity of each 1‐junction on the bond graph
1. Relate velocities (flows, 1‐jct.’s) together using 0‐junctions, transformers, and gyrators
    * Use 0‐junctions to sum absolute velocities, yielding relative velocities
1. Attach one‐port elements to appropriate 1‐junctions
1. Simplify the bond graph
    * Eliminate any two‐port 0‐ or 1‐junctions with through power, e.g. both arrows point in the same direction
1. Inspect the complete bond graph
    * Understand how bond graph relates to physical system
    * All components connected to a 1‐junction move at the same velocity
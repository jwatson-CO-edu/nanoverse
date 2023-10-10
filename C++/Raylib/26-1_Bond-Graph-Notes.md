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

&nbsp;

# Causality

* Bonds have associated effort and flow
    - A component can set either the effort on a bond or the flow on a bond – not both
* Causality indicated by the addition of a causal stroke to the end of each bond
    - Flow is determined by the element near the causal stroke
    - Effort determined by the element away from the causal stroke
* Five Types of Causality
    1. Required: Sources
        * Effort sources can determine effort only
        * Flow sources can determine flow only
    1. Restricted: two‐port elements and n‐port junctions
        * Causality for all connected bonds and elements determined by the causality of one connected bond and element
        * 0‐junctions
            - Constant effort, so only one element can set the effort
            - Only one causal stroke will be near the 0‐junction
        * 1‐junctions
            - Constant flow, so only one element can set the flow
            - All causal strokes, except for one, will be near the 1‐junction
        * Transformers
            - Effort/flow at one port determines effort/flow at the other
            - If TF determines effort at one port, it will determine flow at the other
            - Only one causal stroke near the TF
        * Gyrators
            - Effort at one port determines flow at the other and vice‐versa
            - GY will determine both efforts or both flows
            - Both or neither causal strokes near the GY
    1. Integral: Independent energy‐storage elements (I's and C's). A component in integral causality will either:
        * Integrate effort to determine flow, or
        * Integrate flow to determine effort
        * Energy storage not directly tied to – not algebraically determined by – any other energy‐storage element
        * Elements that are not independent
            - Inertias in integral causality integrate applied effort to determine flow
            - Capacitors in integral causality integrate applied flow to determine effort
    1. Derivative: Dependent energy‐storage elements:
        * A component in derivative causality will either:
            - Differentiate effort to determine flow, or
            - Differentiate flow to determine effort
        * Energy storage directly tied to – algebraically related to – another energy‐storage element
        * Dependent energy storage elements:
            - Inertias in derivative causality differentiate applied flow to determine effort
            - Capacitors in derivative causality differentiate applied effort to determine flow
    1. Arbitrary: Resistors
        * Causality assigned to resistors is determined by the rest of the system
        * Resistors can determine effort from an applied flow
        * Resistors can determine flow from an applied effort 

## Assigning Causality
Starting with a simplified bond graph system model, assign causality to each element

1. Pick a source and assign its required causality 
    - Follow through with any implicated restricted causal assignments (i.e. at 0‐jct., 1‐jct., TF, GY), extending these through the bond graph as far as possible
    - Repeat for all unassigned sources

1. Pick an energy‐storage element (I or C) and assign integral (i.e. preferred) causality
    - Follow through with any implicated restricted causal assignments (i.e. at 0‐jct., 1‐jct., TF, GY), extending these through the bond graph as far as possible
    - Repeat for all unassigned energy‐storage elements

1. If not done: Pick an unassigned resistor, and arbitrarily assign causality
    - Follow through with any implicated restricted causal assignments (i.e. at 0‐jct., 1‐jct., TF, GY), extending these through the bond graph as far as possible
    - Repeat for all unassigned resistors

* Possible Outcomes:
    1. All energy‐storage elements in integral causality (All causality assigned following step 2)
    1. Causality assignment completed by arbitrarily assigning causality of some R‐elements
    (Indicates the presence of algebraic loops or resistor fields)
    1. Some energy‐storage elements forced into derivative causality in step 2
    1. Combination of 2 and 3, algebraic loops and derivative causality

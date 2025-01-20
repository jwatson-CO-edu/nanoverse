# Rampart 2: Wreckage Reckoning
Example Java project exemplifies OOP design patterns in a reimagining of an arcade/NES classic.
## Features
* Keyboard and Gamepad input
## `DEV_PLAN`
* `[Y]` Keyboard input, 2025-01-20: Arrow keyes move the cursor!
    - `[Y]` Move cursor around by units, 2025-01-20: Arrow keyes move the cursor!
    - `[N]` How to draw? (How to negate color below?), 2025-01-20: Use a hollow rect, do not sample color!
* `[ ]` Gamepad input
## Refactor to Patterns
1. `[Y]` 2025-01-XX: `Tile`s are now instantiated by a Factory Pattern
1. `[ ]` Singletons
    - `[ ]` Game Engine
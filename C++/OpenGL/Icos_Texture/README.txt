#############################
README.txt
James Watson, 2018 October
Building, Running, and Usage directions for Homework submission
#############################

=== Build Instructions ===

In the root directory for the project:
source build_HW.sh

The script will:
	1. Create a 'build' directory within the present directory, if it does not exist.  Then cd to 'build'
	2. Run: 'cmake ../' 
	3. Run : make -j4 , then cd to the top program directory
	4. Output the program in the top program directory, named "HW${N}" where '${N}' is the correct HW number

Dependencies:
	* CMake version 3.0+
	* C++11

___ End Build ___


=== Run Instructions ===

In the root directory for the project:
./HW${N}
where '${N}' is the correct HW number

___ End Run ___


=== Usage Instructions ===

~~ Keys ~~

~ This scene has two lighting presets ~

[p] : Pretty Mode  - Plenty of diffuse from the orbiting source plus a little specular, Objects slightly shiny (Scene starts in this setting)
[o] : October Mode - A soft eerie glow emits from the core, illuminating the inner surfaces of the scene. Orbiting source is very dim
	NOTE: Core emits beams of mysterious power at emissivity >= 50

~ Camera Control ~

[Arrow Up/Dn] : Pitch the plot on a horizontal axis parallel to the screen
[Arrow Rt/Lf] : Yaw the plot about lab Z axis
[PgDn/PgUp] : _ Zoom in and out
[0] : _________ Set view angles to default
NOTE: The orthographic view is no longer featured
NOTE: Camera always faces [0,0,0]

~ Light Position ~

[m] : ______ Toggles auto light movement
 [   ]  : __ Lower/rise light (orbit elevation)
 '   \  : __ Move negative / positive theta (orbit angle)
[Home/End] : Move the light source farther from / closer to the center of the scene

~ Light Properties ~

[a/A] : Decrease/increase ambient light from the orbiting light
[d/D] : Decrease/increase diffuse light from the orbiting light
[s/S] : Decrease/increase specular light from the orbiting light
[n/N] : Decrease/increase shininess of all objects except the orbiting light
[e/E] : Decrease/increase emitted light from the core
	NOTE: Core surface emission also controls the diffuse light from the core

~ Other ~

[b] : _________ Toggle mysterious beams
[Esc] : _______ Close program 

___ End Usage ___


=== Dev Time ===

6 hours
	* Finding and applying textures
	* Some math for taking random triangular samples from textures

___ End Time ___

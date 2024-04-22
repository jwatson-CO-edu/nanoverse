#############################
README.txt
James Watson, 2018 September
Building, Running, and Usage directions for Homework submission
#############################

=== Build Instructions ===

Dependencies:
	* CMake version 2.6+
	* C++11

In the root directory for the project:
source build_HW.sh

The script will:
	1. Create a 'build' directory within the present directory, if it does not exist.  Then cd to 'build'
	2. Run: 'cmake ../' 
	3. Run : make -j4 , then cd to the top program directory
	4. Output the program in the top program directory, named "HW${N}" where '${N}' is the correct HW number

___ End Build ___


=== Run Instructions ===

In the root directory for the project:
./HW${N}
where '${N}' is the correct HW number

___ End Run ___


=== Usage Instructions ===

~~ Keys ~~

[Arrow Up/Dn] : Pitch the plot on a horizontal axis parallel to the screen
[Arrow Rt/Lf] : Yaw the plot about its own Z axis
[0] : _________ Set view angles to default

[c] : _________ Toggle joint control AUTO / MANUAL
					AUTO : _ All joints continuously spin at a constant angular speed
					MANUAL : Joints are controlled by the user
[1-6] : _______ MANUAL - Select robot joint to edit , Joints are numbered 1 to 6 - proximal to distal
[n] : _________ MANUAL - Select NO joint , [+/-] have no effect
[+/-] : _______ MANUAL - Increment / Decrement the joint angle indicated by the manual mode
[z] : _________ MANUAL - Set all joint angles to zero

[Esc] : _______ Close program

___ End Usage ___


=== Dev Time ===

16 hours? 
	* I spent some time figuring out how to properly write a CMakeLists.txt
	* Dusted off and templated an old vector math file
	* Class implementing nested reference frames according to Denavit–Hartenberg parameters (Hollerbach formulation)
	* Implementing geo primitives because requirements
	* Tuning draw functions to make robot links aligned to frames and pretty
	* Made scaled compound creatures because requirements

___ End Time ___

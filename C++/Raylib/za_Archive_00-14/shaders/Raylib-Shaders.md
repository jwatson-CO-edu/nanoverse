# Raylib Shaders

## [OpenGL Shaders](https://learnopengl.com/Getting-started/Shaders)
* Shaders are nothing more than programs transforming inputs to outputs.
* Shaders are also very isolated programs in that they're not allowed to communicate with each other; other than the desginated I/O
* Each input variable is also known as a attribute. 
* Data Types
    - `vec`**n**: the default vector of n floats.
    - `bvec`**n**: a vector of n booleans.
    - `ivec`**n**: a vector of n integers.
    - `uvec`**n**: a vector of n unsigned integers.
    - `dvec`**n**: a vector of n double components.
* Components of a vector can be accessed via `vec.x` where `x` is the first component of the vector. 
    - You can use `.x`, `.y`, `.z` and `.w` to access their first, second, third and fourth component respectively. 
    - GLSL also allows you to use `rgba` for colors or `stpq` for texture coordinates, accessing the same components. 
    - You can use any combination of up to 4 letters to create a new vector (of the same type) as long as the original vector has those components
* I/O
    -  When the types and the names are equal on both sides OpenGL will link those variables together and then it is possible to send data between shaders (this is done when linking a program object).
    - Vertex Shader
        * receives its input straight from the vertex data
        * We specify the input variables with location metadata so we can configure the vertex attributes on the CPU. The vertex shader thus requires an extra layout specification for its inputs so we can link it with the vertex data. 
    - Fragment Shader
    - Uniforms
        * A way to pass data from our application on the CPU to the shaders on the GPU.
        * Uniforms are global


&nbsp;

## [Raylib Shader Tutorial](https://www.youtube.com/watch?v=8205iyicv5k)

&nbsp;

## https://bedroomcoders.co.uk/shaders-with-raylib/

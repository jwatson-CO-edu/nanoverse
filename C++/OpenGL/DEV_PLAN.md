# Dependencies
* GLM
    - `cd /tmp`
    - `git clone https://github.com/g-truc/glm.git`
    - `cd glm`
    - `cmake -DGLM_BUILD_TESTS=OFF -DBUILD_SHARED_LIBS=OFF -B build .`
    - `cmake --build build -- all`
    - `sudo cmake --build build -- install`

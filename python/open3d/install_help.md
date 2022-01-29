# Install Required CMake Version
1. `sudo apt remove --purge cmake`
1. `hash -r`
1. `sudo apt install build-essential libssl-dev`
1. `cd /tmp`
1. `wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz`
1. `tar -zxvf cmake-3.20.2.tar.gz`
1. `cd cmake-3.20.2`
1. `./bootstrap`
1. `make -j4`
1. `sudo make install`
1. `cmake --version`, confirm "3.20.2"

# Build Open3D from Source
1. `cd /tmp`
1. `git clone --recursive https://github.com/intel-isl/Open3D`
1. `cd Open3D`
1. `source ./util/install_deps_ubuntu.sh`,  
Enter sudo password and say yes to all prompts
1. `mkdir build && cd $_`
1. `cmake ..`
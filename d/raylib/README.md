# Install Raylib (Ubuntu/Debian)
1. `sudo apt install libasound2-dev mesa-common-dev libx11-dev libxrandr-dev libxi-dev xorg-dev libgl1-mesa-dev libglu1-mesa-dev`
1. `cd /tmp`
1. `git clone https://github.com/raysan5/raylib.git raylib`
1. `cd raylib`
1. `mkdir build && cd build`
1. `cmake -DBUILD_SHARED_LIBS=ON ..`
1. `make -j2`
1. `sudo make install`
1. `sudo reboot now`

# Dlang Raylib API
1. Install Dlang
1. Navigate to project root directory
1. `mkdir source`
1. `touch source/app.d`
1. Code
1. `dub add raylib-d`
1. `dub build`
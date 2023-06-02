1. `sudo apt install libasound2-dev mesa-common-dev libx11-dev libxrandr-dev libxi-dev xorg-dev libgl1-mesa-dev libglu1-mesa-dev`
1. `cd /tmp/`
1. `git clone https://github.com/raysan5/raylib.git raylib`
1. `cd raylib/`
1. `git checkout 4.2.0`
1. `cd src/`
1. `make PLATFORM=PLATFORM_DESKTOP RAYLIB_LIBTYPE=SHARED`
1. `sudo make install RAYLIB_LIBTYPE=SHARED`
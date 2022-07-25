# Install Dependencies 
1. `sudo apt install libgles2-mesa-dev libvulkan-dev glslang-dev`
1. `sudo apt install libsdl2-dev libxt-dev libxaw7-dev doxygen`
1. `git clone https://github.com/zeux/pugixml.git`, Put it somewhere safe
1. `sudo apt install libsdl2-2.0 zlib1g libfreetype6 swig libfreeimage-dev doxygen assimp-utils`
1. `wget -qO - http://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo apt-key add -`
1. `sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-focal.list http://packages.lunarg.com/vulkan/lunarg-vulkan-focal.list`
1. `sudo apt update`
1. `sudo apt install vulkan-sdk`
# Build OGRE
1. Navigate to the downloaded source
1. `mkdir build && cd $_`
1. `cmake ..`
1. `sudo make install -j4`
1. If SWIG was installed before building OGRE, the Python API (`ogre-python`) will automatically become available under Python 3.8. (2022-07-21), `import Ogre`

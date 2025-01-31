# OpenCV
1. Install dependencies
    * `sudo apt install libgtk2.0-dev pkg-config`
1. Nuke all previous installations!
    * `sudo rm /usr/{bin,lib}/*opencv*`, 
    * `sudo rm /usr/local/{bin,lib}/*opencv*`
    * `sudo rm -rf /usr/local/include/*opencv*`
1. `cd /tmp/`
1. Fetch the latest **stable** version of OpenCV and  `opencv_contrib`
    1. `wget -O opencv.zip https://github.com/opencv/opencv/archive/4.9.0.zip`
    1. `unzip opencv.zip`
    1. `git clone https://github.com/opencv/opencv_contrib.git`
    1. `cd opencv_contrib/`
    1. `git checkout 4.9.0`
1. Build OpenCV with extra libraries
    1. `cd ../opencv-4.9.0/`
    1. `mkdir -p build && cd build`
    1. `sudo mkdir /usr/local/include/bullet` , This is a hack for the sake of `opencv_contrib` version 4
    1. `cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_GENERATE_PKGCONFIG=ON .. | tee CMakePrepInfo.txt`
    1. `cat CMakePrepInfo.txt | grep opencv_contrib` , Verify that extra modules are going to be built
    1. `make -j7 | tee makeBuildInfo.txt`
    1. `cat makeBuildInfo.txt | grep xfeatures2d` , Verify that extra 2D features module was **actually** *built*
1. Install OpenCV and Notify the Linker
    1. `sudo make install`
    1. `sudo ldconfig -v` , Let the linker find OpenCV. Needed?
    1. `ls /usr/local/include/opencv4/opencv2/ | grep xfeatures2d` , Verify that extra 2D features module was **actually** *installed*
    1. `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib` , Let the compiler find the shared object location
    1. Add `/usr/local/include/opencv4/` to the editor search path for this project.
1. Build and Install MatIO
    1. `cd /tmp/`
    1. `git clone git://git.code.sf.net/p/matio/matio`
    1. `cd matio`
    1. `git submodule update --init  # for datasets used in unit tests`
    1. `./autogen.sh`
    1. `./configure`
    1. `make`
    1. `make check`
    1. `sudo make install`
1. Build and Install MatIO_Cpp
    1. `cd /tmp/`
    1. `git clone https://github.com/ami-iit/matio-cpp`
    1. `cd matio-cpp`
    1. `mkdir build && cd build`
    1. `cmake ..`
    1. `make`
    1. `sudo make install`
    
<!-- # Eigen3
1. `sudo apt install libeigen3-dev`
1. `cd /usr/include`
1. `sudo ln -sf eigen3/Eigen Eigen`
1. `sudo ln -sf eigen3/unsupported unsupported` -->
# OpenCV
**WARNING: This will take at least an hour to build!**
1. Install dependencies
    * `sudo apt install libgtk2.0-dev libgtk-3-dev pkg-config`
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
    1. `cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D PKG_CONFIG_PATH=/usr/lib/pkgconfig -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D WITH_QT=ON -D WITH_GTK=OFF .. | tee CMakePrepInfo.txt`
    1. `cat CMakePrepInfo.txt | grep opencv_contrib` , Verify that extra modules are going to be built
    1. `unbuffer make -j6 | tee makeBuildInfo.txt`
    1. `cat makeBuildInfo.txt | grep xfeatures2d` , Verify that extra 2D features module was **actually** *built*
1. Install OpenCV and Notify the Linker
    1. `sudo make install`
    1. `sudo ldconfig -v` , Let the linker find OpenCV. Needed?
    1. `ls /usr/local/include/opencv4/opencv2/ | grep xfeatures2d` , Verify that extra 2D features module was **actually** *installed*
    1. `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib` , Let the compiler find the shared object location
    1. Add `/usr/local/include/opencv4/` to the editor search path for this project.

# Point Cloud Library (PCL)
**WARNING: This will take more than an hour to build!**
1. `sudo add-apt-repository ppa:ubuntu-toolchain-r/test && sudo apt update`
1. `sudo apt install gcc-10 gcc-10-base gcc-10-doc g++-10 libstdc++-10-dev libstdc++-10-doc`
1. `sudo apt install libboost-all-dev qtcreator libvtk9-dev libvtk9-qt-dev libeigen3-dev liblz4-dev libqhull-dev expect`
## Eigen3 (C++ Linear Algebra)
1. `cd /usr/include`
1. `sudo ln -sf eigen3/Eigen Eigen`
1. `sudo ln -sf eigen3/unsupported unsupported`
## FLANN
1. `cd /tmp/`
1. `git clone https://github.com/flann-lib/flann.git`
1. `cd flann`
1. `git checkout 1.9.2`
1. `mkdir build && cd $_`
1. `unbuffer cmake .. | tee CMakeInfo.txt`
1. `unbuffer make -j6 | tee makeBuildInfo.txt`
1. `sudo make install`
## PCL
**WARNING: This will take at least an hour to build! (Depending on your system)**
1. `sudo swapoff -a`, Otherwise the session will slow down massively
1. `cd /tmp/`
1. `wget https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/source.tar.gz`
1. `tar xvf source.tar.gz`
1. `cd pcl/`
1. `mkdir build && cd $_`
1. `unbuffer cmake .. | tee CMakeInfo.txt`
1. `unbuffer make -j6 | tee makeBuildInfo.txt`
1. `sudo make install`

## Finish
1. `sudo ldconfig -v`
1. Add to include paths: "/usr/include/vtk-9.1/"
1. Add to include paths: "/usr/local/include/pcl-1.14/"

# COLMAP
Version: 2025-02-25  
## gtest
1. `cd /tmp`
1. `git clone https://github.com/google/googletest.git`
1. `cd googletest/`
1. `git checkout v1.16.0`
1. `mkdir build && cd $_`
1. `unbuffer cmake .. | tee CMakeOut.txt`
1. `unbuffer make -j4 | tee MakeOut.txt`
1. `sudo make install`
## gflags
1. `cd /tmp`
1. `git clone https://github.com/gflags/gflags.git`
1. `cd gflags`
1. `git checkout v2.2.2`
1. `mkdir build && cd $_`
1. `unbuffer cmake .. | tee CMakeOut.txt`
1. `unbuffer make -j4 | tee MakeOut.txt`
1. `sudo make install`
## glog
1. `cd /tmp`
1. `git clone https://github.com/google/glog.git`
1. `cd glog`
1. `git checkout v0.7.0`
1. `mkdir build && cd $_`
1. `unbuffer cmake -DBUILD_SHARED_LIBS=OFF .. | tee CMakeOut.txt`, This flag prevents the `-fPIC` error!
1. `unbuffer make -j4 | tee MakeOut.txt`
1. `sudo make install`
## Other Dependencies
1. `sudo apt install nvidia-cuda-toolkit-gcc libcgal-dev libceres-dev libmpfr-dev libgmp-dev libmetis-dev libfreeimage-dev`
## COLMAP (Finally!)
### NOTE: You may need to upgrade gcc/++ first
1. `sudo add-apt-repository ppa:ubuntu-toolchain-r/test`
1. `sudo apt update`
1. `sudo apt install gcc-13 g++-13`
### Normal Linux Install
1. `cd /tmp`
1. `git clone https://github.com/colmap/colmap.git`
1. `cd colmap`
1. `git checkout 3.11.1`
1. `mkdir build && cd $_`
1. `export CC=/usr/bin/gcc-13`
1. `export CXX=/usr/bin/g++-13`
1. `export CUDA_ROOT=/usr/lib/cuda`, NOTE: This may be different on your distro!
1. `nvidia-smi --query-gpu=compute_cap --format=csv`, Choose the highest one?
1. `unbuffer cmake .. -DCMAKE_CUDA_ARCHITECTURES=75 | tee CMakeOut.txt`
1. `unbuffer make -j4 | tee MakeOut.txt`
1. `sudo make install`

## Finish
1. `sudo ldconfig -v`


# Suspended
**WARNING**: Interoperability with MATLAB is a low priority! Not useful or fun!!
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
    

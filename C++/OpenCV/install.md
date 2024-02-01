1. `cd /tmp/`
1. `wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip`
1. `unzip opencv.zip`
1. `git clone https://github.com/opencv/opencv_contrib.git`
1. `cd opencv-4.x/`
1. `mkdir -p build && cd build`
1. `sudo mkdir /usr/local/include/bullet` , This is a hack for the sake of `opencv_contrib` version 4
1. `cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..`
1. `cmake --build . -- -j 6`
1. `sudo make install`
1. `sudo ldconfig -v`
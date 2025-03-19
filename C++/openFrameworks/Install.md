[Instructions](https://github.com/openframeworks/openFrameworks/blob/master/INSTALL_FROM_GITHUB.md#how-to-compile-openframeworks-from-github)
1. `sudo apt install libglm-dev`
1. `sudo ldconfig -v`
1. `cd /tmp`
1. `git clone --recursive git@github.com:openframeworks/openFrameworks.git --depth 1`
1. `cd openFrameworks/`
1. `git submodule init && git submodule update`
1. `cd scripts/linux/ubuntu`
1. `sudo ./install_dependencies.sh`
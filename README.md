# lc_loam
> Install dependencies
本项目依赖Eigen3,pcl,lua以及google开源的ceres,abseil, glog, gflags
```shell
sudo apt-get install google-mock libboost-all-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev ninja-build stow
```
```shell
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
git checkout d902eb
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl
```

```shell
git clone https://github.com/Thiagoyh/ceres-solver_install.git
cd ceres-solver_install
mkdir build
cd build
cmake .. -G Ninja -DCXX11=ON
ninja
#CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install
```

> 编译运行
```shell
gitt clone https://github.com/Thiagoyh/lc_loam
cd lc_loam
catkin_make install
source install setup.bash
roslaunch lc_loam kitti.launch
```

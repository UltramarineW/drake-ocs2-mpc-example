# [drake-ocs2-mpc-example](https://github.com/UltramarineW/drake-ocs2-mpc-example)
Using drake for dynamic simulation and multibody kinematic computation and use ocs2 for model predict control solution without ros framework. The simplest example double integrator is given in the code. 

## Running Example

If you don't have drake installed, you need to install the drake library first. You can get more information from [drake](https://drake.mit.edu/). Since ocs2 uses the catkin package, you will also need to install ros noetic to compile the module.

``` bash
cd drake-ocs2-mpc-example
mkdir build
cd build
cmake ..
make -j
./src/mpc_double_integrator/mpc_double_integrator
```

Use meshcat slide bar to modify desire target position

![meshcat_slider](https://https://ultramarine-image.oss-cn-beijing.aliyuncs.com/img/1691735025799.png)

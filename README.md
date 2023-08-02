# drake-ocs2-mpc-example
Using drake for dynamic simulation and multibody kinematic computation and use ocs2 for model predict control solution without ros framework.  

## Running Example

Need to modify the path exists in `./src/mpc_double_integrator/mpc_double_integrator.cpp`

``` bash
cd drake-ocs2-mpc-example
mkdir build
cd build
cmake ..
make -j
./src/mpc_double_integrator/mpc_double_integrator
```

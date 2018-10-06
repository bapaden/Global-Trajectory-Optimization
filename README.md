# Global Trajectory Optimization via the Generalized Label Correcting Method


This is a library for solving global trajectory optimization problems given:

1) A dynamical model for the system (i.e. x'(t)=f(x(t),u(t)) 

2) A cost function of a trajectory x(t) and input signal u(t) to be minimized ( i.e. C(x,u)= Integral g(x(t),u(t)) )

3) A collision detector. That is, if some subset of the state space must be avoided, then function must be provided to determine if a trajectory intersects that region

4) An admissible heuristic to be used in a graph search. Note that the heuristic h(x)=0 is always admissible.

This library may be considered as an alternative to 
a) Sampling-based planners
b) Variational trajectory optimization utilizing nonlinear programming techniques

Pros/Cons over sampling-based planners: this method handles differential constraints that arise in nonholonomic planning and planning for complex dynamical systems. The reason is that optimal sampling-based planners (such as RRT*) require a steering subroutine in addition to (1)-(4) above which is non-trivial to provide in general. The downside is that for holonomic planning, such as simple shortest path queries, RRT* and PRM* achieve better performance in comparable implementations.

Pros/Cons over variational trajectory optimization: this method is far more robust than nonlinear programming based techniues. It does not require a "good initial guess" and it will not converge to a locally optimal solution. The downside is the complexity of this method is exponential with the state space. Up to 3 dimensional state spaces are dealt with very well. In higher dimensions, some domain knowledge will be required to construct a good heuristic.

### Documentation

See the included paper from the Workshop on the Algorithmic Foundations of Robotics (2016) for the theoretical analysis.  Documentation of the C++ implementation can be found here:

[https://codedocs.xyz/bapaden/Global-Trajectory-Optimization/md_README.html](https://codedocs.xyz/bapaden/Global-Trajectory-Optimization/md_README.html)

### Installation

To install the library after downloading the source code, enter the following terminal commands from the top level directory

```
mkdir build
cd build
cmake ..
make
make test 
sudo make install
```

The unit tests are run using GTest. If it is not installed in your image you can install it with apt, or compile from source

```
sudo apt-get install libgtest-dev
```

[https://github.com/google/googletest](https://github.com/google/googletest)

### Running the examples

Several basic examples demonstrating how to interface with the library can ve found in the examples/ directory. Each example generates data that is saved in the plots/ directory which contains python scripts to generate basic illustrations of the solution. To run the examples:

```
cd GlobalTrajectoryOptimization/build/examples/
./shortest-path-demo
./pendulum-swingup-demo
./nonholonomic-car-demo
```

To view the solutions:

```
cd GlobalTrajectoryOptimization/plots/
python 
```


### API 

Using Cmake, you can link to the installed library as follows:

```
find_package(glc)
include_directories(${GLC_INCLUDE_DIRS})

add_executable(your_awesome_planning_algorithm your_src.cpp)
target_link_libraries(your_awesome_planning_algorithm glc_planner_core)
```

In your source code where you instantiate a Planner object include the header
```
#include<glc_planner_core.h>
```

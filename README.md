# Global Trajectory Optimization via the Generalized Label Correcting Method


Explain what the lib does and point to reference

### Documentation

Doxygen generated documentation is hosted here:

[https://codedocs.xyz/bapaden/Global-Trajectory-Optimization/md_README.html](https://codedocs.xyz/bapaden/Global-Trajectory-Optimization/md_README.html)

### Compiling and Installation

To install the library, enter the following terminal commands from the top level directory

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

### API 

Using Cmake, you can link to the installed library as follows:

```
...
find_package(glc)
include_directories(${GLC_INCLUDE_DIRS})

add_executable(your_awesome_planning_algorithm your_src.cpp)
target_link_libraries(your_awesome_planning_algorithm glc_planner_core)
...
```

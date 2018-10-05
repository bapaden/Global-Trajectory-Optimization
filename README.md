# Global Trajectory Optimization via the Generalized Label Correcting Method

Explain what the lib does and point to reference

## Compiling and Installation

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

cmake_minimum_required(VERSION 3.5)
project(glc)

find_package(GTest REQUIRED)
enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -std=c++0x -fopenmp -O3")

# Set variables
include(${CMAKE_SOURCE_DIR}/cmake/SetEnv.cmake)

# Source for library
add_subdirectory(glc)

# User interface examples
add_subdirectory(examples)

# Unit tests
add_subdirectory(test)

# Install targets
include(${CMAKE_SOURCE_DIR}/cmake/InstallConfig.cmake)


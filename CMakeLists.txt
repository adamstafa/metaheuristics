cmake_minimum_required(VERSION 3.10)

project(inf273 VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Add subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/src)

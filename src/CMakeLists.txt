cmake_minimum_required(VERSION 3.10)

project(inf273)

set(CMAKE_CXX_STANDARD 23)

# Add source files
file(GLOB SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.hpp"
)

# Rename the binary to 'optimize' and build it at the top of the build directory
add_executable(optimize ${SOURCES})
set_target_properties(optimize PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

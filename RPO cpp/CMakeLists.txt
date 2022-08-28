# Set the minimum CMake version
cmake_minimum_required(VERSION 2.8.12)

# Set the project name
project (RANDOM_PARTICLE_OPTIMIZATION)

# Set build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add library subdirectory
set(RANDOM_PARTICLE_OPTIMIZATION_INCLUDE_RPO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rpo)
add_subdirectory(rpo)
include_directories(${RANDOM_PARTICLE_OPTIMIZATION_INCLUDE_RPO_DIR})

# Add the main file, and name of its executable
add_executable(random_particle_optimization main.cpp)

# Link the library to the main file
target_link_libraries(random_particle_optimization rpo)
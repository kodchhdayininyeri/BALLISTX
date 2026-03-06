cmake_minimum_required(VERSION 3.20)
project(BALLISTX VERSION 0.1.0 LANGUAGES CXX)

# C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MSVC specific flags for C++17
if(MSVC)
    add_compile_options(/std:c++17 /Zc:__cplusplus)
endif()

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files - will be populated as we add modules
set(BALLISTX_SOURCES
    src/ballistics/projectile.cpp
    src/atmosphere/isa_model.cpp
    src/aerodynamics/drag_model.cpp
)

# Create library
add_library(ballistx ${BALLISTX_SOURCES})

# Examples executable
add_executable(ballistx_demo examples/demo.cpp)
target_link_libraries(ballistx_demo PRIVATE ballistx)

# Simple trajectory example
add_executable(simple_trajectory examples/simple_trajectory.cpp)
target_link_libraries(simple_trajectory PRIVATE ballistx)

# RK4 trajectory example
add_executable(rk4_trajectory examples/rk4_trajectory.cpp)
target_link_libraries(rk4_trajectory PRIVATE ballistx)

# Mach drag demo
add_executable(mach_drag_demo examples/mach_drag_demo.cpp)
target_link_libraries(mach_drag_demo PRIVATE ballistx)

# Gravity demo
add_executable(gravity_demo examples/gravity_demo.cpp)
target_link_libraries(gravity_demo PRIVATE ballistx)

# Quaternion demo
add_executable(quaternion_demo examples/quaternion_demo.cpp)
target_link_libraries(quaternion_demo PRIVATE ballistx)

# Tests executable
enable_testing()
add_executable(ballistx_tests tests/main_tests.cpp)
if(TARGET ballistx)
    target_link_libraries(ballistx_tests PRIVATE ballistx)
endif()

# Print configuration
message(STATUS "BALLISTX v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

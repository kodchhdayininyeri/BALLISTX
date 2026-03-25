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
    src/guidance/guidance.cpp
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

# 6-DOF state demo
add_executable(state_6dof_demo examples/state_6dof_demo.cpp)
target_link_libraries(state_6dof_demo PRIVATE ballistx)

# Magnus effect demo
add_executable(magnus_demo examples/magnus_demo.cpp)
target_link_libraries(magnus_demo PRIVATE ballistx)

# Coriolis effect demo
add_executable(coriolis_demo examples/coriolis_demo.cpp)
target_link_libraries(coriolis_demo PRIVATE ballistx)

# Wind model demo
add_executable(wind_model_demo examples/wind_model_demo.cpp)
target_link_libraries(wind_model_demo PRIVATE ballistx)

# Projectile types demo
add_executable(projectile_types_demo examples/projectile_types_demo.cpp)
target_link_libraries(projectile_types_demo PRIVATE ballistx)

# Guidance demo
add_executable(guidance_demo examples/guidance_demo.cpp)
target_link_libraries(guidance_demo PRIVATE ballistx)

# Proportional Navigation demo
add_executable(proportional_navigation_demo examples/proportional_navigation_demo.cpp)
target_link_libraries(proportional_navigation_demo PRIVATE ballistx)

# Target demo
add_executable(target_demo examples/target_demo.cpp)
target_link_libraries(target_demo PRIVATE ballistx)

# Miss distance demo
add_executable(miss_distance_demo examples/miss_distance_demo.cpp)
target_link_libraries(miss_distance_demo PRIVATE ballistx)

# EKF demo
add_executable(ekf_demo examples/ekf_demo.cpp)
target_link_libraries(ekf_demo PRIVATE ballistx)

# Kalman visualization demo
add_executable(kalman_visualization_demo examples/kalman_visualization_demo.cpp)
target_link_libraries(kalman_visualization_demo PRIVATE ballistx)

# ========================================================================
# Unit Tests with Google Test
# ========================================================================
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    # Include FetchContent module
    include(FetchContent)

    # Download and configure Google Test
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )

    # For Windows: Prevent overriding parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(googletest)

    # Disable compiling gtest's own tests
    set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)

    message(STATUS "Google Test configured via FetchContent")

    # Create test executable
    add_executable(ballistx_tests
        tests/main_tests.cpp
        tests/test_vec3.cpp
        tests/test_quaternion.cpp
        tests/test_projectile.cpp
        tests/test_guidance.cpp
    )

    # Kalman filter test executable (standalone, no GoogleTest)
    add_executable(kalman_filter_test tests/test_kalman_filter.cpp)
    target_link_libraries(kalman_filter_test PRIVATE ballistx)

    # Radar model test executable (standalone, no GoogleTest)
    add_executable(radar_model_test tests/test_radar_model.cpp)
    target_link_libraries(radar_model_test PRIVATE ballistx)

    # IMU model test executable (standalone, no GoogleTest)
    add_executable(imu_model_test tests/test_imu_model.cpp)
    target_link_libraries(imu_model_test PRIVATE ballistx)

    # Sensor fusion test executable (standalone, no GoogleTest)
    add_executable(sensor_fusion_test tests/test_sensor_fusion.cpp)
    target_link_libraries(sensor_fusion_test PRIVATE ballistx)

    target_link_libraries(ballistx_tests
        PRIVATE
        ballistx
        gtest
        gtest_main
    )

    target_include_directories(ballistx_tests
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )

    # Discover tests
    enable_testing()
    include(GoogleTest)
    gtest_discover_tests(ballistx_tests)

    message(STATUS "Unit tests configured - run with: cmake --build build --target test")
endif()

# ========================================================================
# Python Bindings with pybind11
# ========================================================================
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

if(BUILD_PYTHON_BINDINGS)
    # Find Python
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    # Try to find pybind11 from pip installation
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    if(DEFINED PYBIND11_CMAKE_DIR AND EXISTS "${PYBIND11_CMAKE_DIR}")
        message(STATUS "Found pybind11 cmake dir: ${PYBIND11_CMAKE_DIR}")
        list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
    endif()

    # Try to find pybind11
    find_package(pybind11 QUIET)

    if(pybind11_FOUND)
        message(STATUS "pybind11 found: ${pybind11_VERSION}")

        # Create Python module (output name will be ballistx.pyd)
        pybind11_add_module(ballistx_module src/bindings/bindings.cpp)

        # Link against the ballistx library
        target_link_libraries(ballistx_module PRIVATE ballistx)

        # Include directories
        target_include_directories(ballistx_module PRIVATE ${PROJECT_SOURCE_DIR}/include)

        # Set output directory and module name
        # Python module will be importable as "ballistx"
        set_target_properties(ballistx_module PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/modules
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/modules
            OUTPUT_NAME "_ballistx"
            PREFIX ""
        )

        message(STATUS "Python bindings will be built to: ${CMAKE_BINARY_DIR}/python")
    else()
        message(WARNING "pybind11 not found via CMake. Trying manual configuration...")

        # Manual configuration if find_package fails
        if(DEFINED PYBIND11_CMAKE_DIR)
            # Add pybind11 include directory manually
            execute_process(
                COMMAND ${Python3_EXECUTABLE} -c "import pybind11; import os; print(os.path.dirname(pybind11.__file__))"
                OUTPUT_VARIABLE PYBIND11_BASE_DIR
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )

            if(EXISTS "${PYBIND11_BASE_DIR}/include")
                # Create Python module manually
                add_library(ballistx_module MODULE src/bindings/bindings.cpp)

                # Link against the ballistx library
                target_link_libraries(ballistx_module PRIVATE ballistx)

                # Include directories
                target_include_directories(ballistx_module PRIVATE
                    ${PROJECT_SOURCE_DIR}/include
                    "${PYBIND11_BASE_DIR}/include"
                    ${Python3_INCLUDE_DIRS}
                )

                # Set output directory
                set_target_properties(ballistx_module PROPERTIES
                    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/modules
                    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/modules
                    PREFIX ""
                    OUTPUT_NAME "_ballistx"
                )

                message(STATUS "Python bindings configured manually")
            endif()
        else()
            message(WARNING "pybind11 not found. Install with: pip install pybind11")
        endif()
    endif()
endif()

# Print configuration
message(STATUS "BALLISTX v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# CMakeLists.txt

# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(kd_tree VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source and include directories
set(SOURCES src/main.cpp)
set(HEADER_DIRS include)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories for the compiler
target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_DIRS})
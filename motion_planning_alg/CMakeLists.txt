cmake_minimum_required (VERSION 3.10)

# Define the project
project(cmake-tutorial)

# GoogleTest requires at least C++17
set(CMAKE_CXX_STANDARD 17)

# add_library(root_finding src/root_finding/)

find_package (Eigen3 REQUIRED)
# include_directories(cmake-tutorial PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
# Add definition for the cmake-tutorial binary
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(motion_planning src/main.cpp src/cubic_spline_todo.cpp)
target_link_libraries(motion_planning Eigen3::Eigen cnpy)

add_executable(lqr_controller src/lqr_controller.cpp src/reference_trajectory_generator_todo.cpp)
target_link_libraries(lqr_controller Eigen3::Eigen cnpy)

# Specify where third-party libraries are located
link_directories(${CMAKE_BINARY_DIR}/lib/installed/${CMAKE_INSTALL_LIBDIR})
include_directories(${CMAKE_BINARY_DIR}/lib/installed/${CMAKE_INSTALL_INCLUDEDIR})
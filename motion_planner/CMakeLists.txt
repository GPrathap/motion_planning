cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17 -std=gnu++1z ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    tf 
    nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl)


set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

link_libraries(${OMPL_LIBRARY_DIRS})

add_executable( run_planner
    src/run_planner.cpp
    src/grid_graph.cpp
    src/planner.cpp
    src/vis_utils.cpp
    src/a_star.cpp)

target_link_libraries(run_planner 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OMPL_LIBRARIES}
)

add_executable (random_complex  src/random_complex_generator.cpp )

target_link_libraries(random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  

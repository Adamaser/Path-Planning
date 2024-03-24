cmake_minimum_required(VERSION 2.8.3)
project(grid_path_searcher)
set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

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
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror


##生成astar_demo节点
add_executable( astar_demo 
    src/astar_demo.cpp
    src/Astar.cpp
    src/planner.cpp
    src/trajectory_generator.cpp
    )
target_link_libraries(astar_demo 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

##生成random_complex点云地图节点
add_executable ( random_complex 
    src/generate_map.cpp )
target_link_libraries( random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  

##生成complex_map点云地图节点
add_executable ( complex_map 
    src/complex_map.cpp )
target_link_libraries( complex_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ) 

cmake_minimum_required(VERSION 3.0.2)
project(ompl_rrtstar)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

find_package(ompl REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_LATEST_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  /usr/include/pcl-1.8
  /usr/include/eigen3
)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

add_executable(rrt_star_planner src/rrt_star_planner.cpp)

target_link_libraries(rrt_star_planner 
  ${OMPL_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

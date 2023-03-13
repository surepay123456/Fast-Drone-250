cmake_minimum_required(VERSION 3.0.2)
project(test_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  plan_env
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 LIBRARIES  test_map_node
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS OpenCV plan_env
#  DEPENDS system_lib
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} #${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# add_library( test_map 
#     src/test_map_node.cpp 
#     )
add_executable(test_map_node src/test_map_node)

target_link_libraries( test_map_node
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

# add_executable(obj_generator
#     src/obj_generator.cpp 
# )
# target_link_libraries(obj_generator 
#     ${catkin_LIBRARIES}
#     )

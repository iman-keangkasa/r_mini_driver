cmake_minimum_required(VERSION 3.0.2)
project(r_mini_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_toolbox
  controller_manager
  diagnostic_updater
  hardware_interface
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

# yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS dynamixel_workbench_toolbox controller_manager diagnostic_updater hardware_interface roscpp cmake_modules
 DEPENDS EIGEN3
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${YAML_CPP_INCLUDE_DIRS}
)

add_library(r_mini_driver src/r_mini_hardware_interface.cpp)
add_library(r_mini_driver_pointer src/r_mini_hardware_interface_pointer.cpp)
#target_link_libraries(r_mini_driver ${catkin_LIBRARIES})
# example include the hw interface 
add_executable(r_mini_driver_node src/r_mini_driver_node.cpp)
add_executable(r_mini_driver_pointer_node src/r_mini_driver_pointer_node.cpp)

add_dependencies(r_mini_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(r_mini_driver_pointer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(r_mini_driver_node r_mini_driver ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(r_mini_driver_pointer_node r_mini_driver_pointer ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

#add_executable()
#add_dependencies( ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries( ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

cmake_minimum_required(VERSION 3.0.2)
project(manta_positioning)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
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

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
message_runtime
roscpp
sensor_msgs
std_msgs
roslib
message_generation
)

add_message_files(
  FILES
  mqtt_msg.msg
  mqtt_mag.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib sensor_msgs  geometry_msgs
  # DEPENDS system_lib
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLIDE_DIRS}
)

add_executable(manta_positioning src/manta_positioning.cpp)
target_link_libraries(manta_positioning ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(manta_positioning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(manta_positioning_association src/manta_positioning_association.cpp)
target_link_libraries(manta_positioning_association ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(manta_positioning_association ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
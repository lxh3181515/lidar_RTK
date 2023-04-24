cmake_minimum_required(VERSION 3.0.2)
project(lidar_RTK)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_compile_options(-std=c++14)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  nav_msgs
  pcl_ros
  std_srvs
  std_msgs
  message_generation
)

add_service_files(
   FILES
   optimizeMap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/include
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/pcl.cmake)
include(cmake/geographic.cmake)
include(cmake/eigen.cmake)
include(cmake/g2o.cmake)
include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/global_defination.cmake)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(loop_detect_node src/apps/loop_detect_node.cpp ${ALL_SRCS})
add_dependencies(loop_detect_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loop_detect_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

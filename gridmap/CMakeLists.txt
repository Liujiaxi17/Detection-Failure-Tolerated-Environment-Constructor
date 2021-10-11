cmake_minimum_required(VERSION 2.8.3)
project(grid_map_maker)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    pcl_ros
    sensor_msgs
    tf
    tf2_msgs
  )

find_package(Boost REQUIRED COMPONENTS system filesystem)


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
    CATKIN_DEPENDS
    std_msgs
    pcl_ros
    sensor_msgs
    tf
    tf2_msgs
  )

include_directories(
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
  ${Eigen3_INCLUDE_DIR}
  ${ZZZ_ROOT}/devel/include
)

#imm_ukf_pda
add_executable(grid_map_maker
  src/gridmap_node.cpp
  src/gridmapconstructor.cpp
  src/rsgridmap.cc
)
target_link_libraries(grid_map_maker
  ${catkin_LIBRARIES} yaml-cpp  
  ${Boost_LIBRARIES}
)

add_dependencies(grid_map_maker
  ${catkin_EXPORTED_TARGETS}
  )


install(TARGETS
grid_map_maker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

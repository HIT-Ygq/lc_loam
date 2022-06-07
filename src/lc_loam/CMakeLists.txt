cmake_minimum_required(VERSION 2.8.3)
project(lc_loam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  last_map.srv
  last_optimization.srv
  save_pcd.srv
  save_trajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(absl REQUIRED)
find_package(Lua REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  LIBRARIES lc_loam
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

set(LC_LOAM_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/lc_loam/configuration_files
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/lc_loam/)
install(DIRECTORY launch DESTINATION share/lc_loam/)
install(DIRECTORY rviz DESTINATION share/lc_loam/)


#message("project_source_dir is: ${PROJECT_SOURCE_DIR}")
configure_file(
    ${PROJECT_SOURCE_DIR}/include/config/config.h.cmake
    ${PROJECT_BINARY_DIR}/config/config.h)
include_directories(${PROJECT_BINARY_DIR})

message(" ")

file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h" "include/*.h" "include/*.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cpp" "src/*_test.cpp")
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})


add_executable(laser_processing_node src/scanRegistration_main.cpp ${ALL_SRCS})
target_link_libraries(laser_processing_node ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            ${EIGEN3_LIBRARIES}
                                            ${CERES_LIBRARIES}
                                            )
add_executable(map_builder_node src/node_main.cpp ${ALL_SRCS})
target_link_libraries(map_builder_node ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
                                            ${EIGEN3_LIBRARIES}
                                            ${CERES_LIBRARIES}
                                                          )
target_link_libraries(laser_processing_node
    absl::algorithm
    absl::base
    absl::debugging
    absl::flat_hash_map
    absl::memory
    absl::meta
    absl::numeric
    absl::str_format
    absl::strings
    absl::synchronization
    absl::time
    absl::utility)

target_link_libraries(map_builder_node
absl::algorithm
absl::base
absl::debugging
absl::flat_hash_map
absl::memory
absl::meta
absl::numeric
absl::str_format
absl::strings
absl::synchronization
absl::time
absl::utility)

target_include_directories(map_builder_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_include_directories(laser_processing_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(map_builder_node  ${LUA_LIBRARIES})
target_link_libraries(laser_processing_node  ${LUA_LIBRARIES})

install(TARGETS map_builder_node laser_processing_node
                                      DESTINATION lib/${PROJECT_NAME})

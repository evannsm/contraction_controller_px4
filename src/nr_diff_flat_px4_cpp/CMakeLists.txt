cmake_minimum_required(VERSION 3.14)
project(nr_diff_flat_px4_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(quad_platforms_cpp REQUIRED)
find_package(ros2_logger_cpp REQUIRED)

# autodiff: try installed version first (from quad_trajectories_cpp), otherwise fetch
find_package(autodiff QUIET)
if(NOT autodiff_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    autodiff
    GIT_REPOSITORY https://github.com/autodiff/autodiff.git
    GIT_TAG v1.1.2
  )
  set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(autodiff)
endif()

find_package(quad_trajectories_cpp REQUIRED)

# Controller library (no ROS deps)
add_library(nr_controller
  src/controller/nr_utils.cpp
  src/controller/newton_raphson.cpp
)
target_include_directories(nr_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(nr_controller PUBLIC
  Eigen3::Eigen
  autodiff::autodiff
)

# Main executable
add_executable(run_node
  src/offboard_control_node.cpp
  src/run_node.cpp
)
target_include_directories(run_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(run_node PRIVATE
  nr_controller
  quad_platforms_cpp::quad_platforms_cpp
  quad_trajectories_cpp::quad_trajectories_cpp
  Eigen3::Eigen
  ros2_logger_cpp::ros2_logger_cpp
  ${rclcpp_LIBRARIES}
  ${px4_msgs_LIBRARIES}
)
target_include_directories(run_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
)

install(TARGETS run_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()

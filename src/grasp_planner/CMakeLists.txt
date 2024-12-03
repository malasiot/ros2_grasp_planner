cmake_minimum_required(VERSION 3.8)
project(grasp_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grasp_planner_interfaces REQUIRED)
find_package(grasp_planner_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED core imgproc)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenMP REQUIRED)
find_package(cvx REQUIRED)


add_executable(grasp_planner_service
    src/grasp_planner.cpp 
    src/viz.cpp 
    src/moveit_ik.cpp 
    src/moveit_ik.hpp
    src/move_group_interface.cpp
    src/move_group_interface.hpp
    src/grasp_planner_service.cpp
    src/grasp_planner_service.hpp
    src/masked_point_cloud.cpp
    src/masked_point_cloud.hpp
)

target_include_directories(grasp_planner_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(grasp_planner_service PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(grasp_planner_service
  rclcpp  std_msgs sensor_msgs geometry_msgs 
  cv_bridge Eigen3 cvx
  visualization_msgs
  image_transport
  image_geometry
  tf2
  tf2_ros tf2_eigen
  moveit_ros_planning_interface
  grasp_planner_msgs
  grasp_planner_interfaces
)
target_link_libraries(grasp_planner_service ${OpenCV_LIBS} Eigen3::Eigen cvx OpenMP::OpenMP_CXX)


add_executable(reachability
    src/reachability.cpp 
    src/reachability_node.hpp
    src/reachability_node.cpp
    src/moveit_ik.cpp 
    src/moveit_ik.hpp
)

target_compile_features(reachability PUBLIC cxx_std_17)
ament_target_dependencies(reachability
  rclcpp  std_msgs sensor_msgs geometry_msgs 
  Eigen3
  moveit_ros_planning_interface
  grasp_planner_interfaces
)



install(TARGETS grasp_planner_service reachability
  DESTINATION lib/${PROJECT_NAME})

  ################################## grasp_candidates_service ###########################################

add_executable(grasp_candidates_service
src/grasp_candidates_service.cpp 
src/viz.cpp 
src/grasp_candidates_node.cpp
src/grasp_candidates_node.hpp
src/masked_point_cloud.cpp
src/masked_point_cloud.hpp
)

target_include_directories(grasp_candidates_service PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${OpenCV_INCLUDE_DIRS})

target_compile_features(grasp_candidates_service PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(grasp_candidates_service
rclcpp  std_msgs sensor_msgs geometry_msgs 
cv_bridge Eigen3 cvx
visualization_msgs
image_transport
image_geometry
tf2
tf2_ros tf2_eigen
grasp_planner_msgs
grasp_planner_interfaces
)
target_link_libraries(grasp_candidates_service ${OpenCV_LIBS} Eigen3::Eigen cvx OpenMP::OpenMP_CXX)


install(TARGETS grasp_candidates_service DESTINATION lib/${PROJECT_NAME})

################################## grasp_candidates_filter_service ###########################################

add_executable(grasp_candidates_filter_service
src/grasp_candidates_filter_service.cpp 
src/viz.cpp 
src/moveit_ik.cpp
src/moveit_ik.hpp
src/grasp_candidates_filter_node.cpp
src/grasp_candidates_filter_node.hpp
)

target_include_directories(grasp_candidates_filter_service PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${OpenCV_INCLUDE_DIRS})

target_compile_features(grasp_candidates_filter_service PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(grasp_candidates_filter_service
  rclcpp  std_msgs sensor_msgs geometry_msgs 
  cv_bridge Eigen3 cvx
  visualization_msgs
  image_transport
  image_geometry
  tf2
  tf2_ros tf2_eigen
  moveit_ros_planning_interface
  grasp_planner_msgs
)
target_link_libraries(grasp_candidates_filter_service Eigen3::Eigen cvx OpenMP::OpenMP_CXX)


install(TARGETS grasp_candidates_filter_service DESTINATION lib/${PROJECT_NAME})

################################################################################################3

install(
  DIRECTORY launch data config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

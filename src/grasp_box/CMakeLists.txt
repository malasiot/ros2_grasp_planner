cmake_minimum_required(VERSION 3.8)
project(grasp_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cvx REQUIRED)
find_package(grasp_planner_interfaces REQUIRED)

add_executable(test_box_detector src/test_box_detector.cpp src/box_detector.cpp src/box_detector.hpp)

target_include_directories(test_box_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CVX_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

target_compile_features(test_box_detector PUBLIC cxx_std_17)  # Require C99 and C++17
target_link_libraries(test_box_detector cvx ${OpenCV_LIBRARIES})

add_executable(grasp_box_service 
    src/grasp_box.cpp
    src/grasp_box_service.cpp 
    src/grasp_box_service.hpp 
    src/box_detector.cpp 
    src/box_detector.hpp)

target_include_directories(grasp_box_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CVX_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

target_compile_features(grasp_box_service PUBLIC cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  grasp_box_service
  rclcpp  std_msgs sensor_msgs geometry_msgs 
  cv_bridge Eigen3
  visualization_msgs
  image_geometry
  tf2_eigen
  cvx
  grasp_planner_interfaces
)

target_link_libraries(grasp_box_service cvx ${OpenCV_LIBRARIES})

install(TARGETS grasp_box_service DESTINATION lib/${PROJECT_NAME})


ament_package()

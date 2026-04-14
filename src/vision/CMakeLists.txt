
cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(yolo_object_detection
  src/yolo_object_detection.cpp
)

target_link_libraries(yolo_object_detection
  depthai::core
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolo_object_detection
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
  geometry_msgs
)

target_include_directories(yolo_object_detection PRIVATE
  /opt/ros/humble/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(target_range_node
  src/target_range_node.cpp
)

ament_target_dependencies(target_range_node
  rclcpp
  std_msgs
  sensor_msgs
  vision_msgs
  cv_bridge
)

target_include_directories(target_range_node PRIVATE
  /opt/ros/humble/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(target_range_node
  ${OpenCV_LIBS}
)

install(TARGETS
  yolo_object_detection
  target_range_node
  DESTINATION lib/${PROJECT_NAME}
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

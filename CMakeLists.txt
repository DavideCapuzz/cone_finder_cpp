cmake_minimum_required(VERSION 3.8)
project(cone_finder_cpp)

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
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(cone_finder src/cone_finder.cpp)
add_executable(visualize_target src/visualize_target.cpp)

ament_target_dependencies(cone_finder 
sensor_msgs
vision_msgs
rclcpp
std_msgs
cv_bridge image_transport OpenCV)

ament_target_dependencies(visualize_target 
sensor_msgs
vision_msgs
rclcpp
std_msgs
cv_bridge image_transport OpenCV)

target_include_directories(cone_finder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cone_finder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
cone_finder
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(visualize_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(visualize_target PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
visualize_target
  DESTINATION lib/${PROJECT_NAME})

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

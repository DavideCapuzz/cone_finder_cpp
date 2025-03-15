cmake_minimum_required(VERSION 3.8)
project(cone_finder_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)  # or 17, depending on your needs
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tools_nav REQUIRED)
find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(HEADERS_EXECUTABLE
    ${YAML_INCLUDE_DIRS}
)

# add_executable(cone_finder src/cone_finder.cpp src/tools.cpp src/core_cone_finder.cpp)
# add_executable(cone_finder_test src/cone_finder_test.cpp src/tools.cpp src/core_cone_finder.cpp)
add_executable(search_target src/search_target.cpp src/tools.cpp src/core_search_target.cpp)
add_executable(search_target_test src/search_target_test.cpp src/tools.cpp src/core_search_target.cpp)

# ament_target_dependencies(cone_finder 
# visualization_msgs
# sensor_msgs
# nav_msgs
# vision_msgs
# rclcpp
# std_msgs
# geometry_msgs
# tf2
# tf2_ros
# tf2_geometry_msgs
# tools_nav
# interfaces
# cv_bridge image_transport OpenCV
# common)

# target_link_libraries(cone_finder
#     yaml-cpp
# )

# ament_target_dependencies(cone_finder_test
# visualization_msgs
# sensor_msgs
# nav_msgs
# vision_msgs
# rclcpp
# std_msgs
# geometry_msgs
# tf2
# tf2_ros
# tf2_geometry_msgs
# tools_nav
# interfaces
# cv_bridge image_transport OpenCV
# common)

# target_link_libraries(cone_finder_test
#     yaml-cpp
# )

ament_target_dependencies(search_target 
visualization_msgs
sensor_msgs
nav_msgs
vision_msgs
rclcpp
std_msgs
geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs
tools_nav
interfaces
cv_bridge image_transport OpenCV
common)

target_link_libraries(search_target
    yaml-cpp
)

ament_target_dependencies(search_target_test
visualization_msgs
sensor_msgs
nav_msgs
vision_msgs
rclcpp
std_msgs
geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs
tools_nav
interfaces
cv_bridge image_transport OpenCV
common)

target_link_libraries(search_target_test
    yaml-cpp
)

# target_include_directories(cone_finder PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(cone_finder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# target_include_directories(cone_finder_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(cone_finder_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(search_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(search_target PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(search_target_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(search_target_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
# cone_finder
# cone_finder_test
search_target
search_target_test
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test test/search_target.cpp)
#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   target_link_libraries(${PROJECT_NAME}_test name_of_local_library)
# endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(cpp_camera_composition)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# cam2image
add_executable(cam2image_node src/cam2image.cpp)
ament_target_dependencies(cam2image_node 
  rclcpp
  std_msgs 
  sensor_msgs
)
target_link_libraries(cam2image_node
  ${OpenCV_LIBS}
)

# showimage
add_executable(showimage_node src/showimage.cpp)
ament_target_dependencies(showimage_node 
  rclcpp
  sensor_msgs
)
target_link_libraries(showimage_node
  ${OpenCV_LIBS}
)

# grayimage
add_executable(grayimage_node src/grayimage.cpp)
ament_target_dependencies(grayimage_node 
  rclcpp
  sensor_msgs
)
target_link_libraries(grayimage_node
  ${OpenCV_LIBS}
)

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node 
  rclcpp
)
target_link_libraries(test_node
  ${OpenCV_LIBS}
)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp 
  example_interfaces
)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp
  example_interfaces
)

# grayimage_component
add_library(grayimage_component SHARED src/grayimage_component.cpp)
target_compile_definitions(grayimage_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(grayimage_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(grayimage_component
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(grayimage_component "cpp_camera_composition::GrayImage")
set(node_plugins "${node_plugins}cpp_camera_composition::GrayImage;$<TARGET_FILE:grayimage_component>\n")

install(TARGETS
  grayimage_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS
    client 
    server
    test_node
    cam2image_node
    showimage_node
    grayimage_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

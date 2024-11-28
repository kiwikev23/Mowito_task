cmake_minimum_required(VERSION 3.8)
project(mowito_task)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

set(srv_files
  "srv/ModeSelect.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(image_converter_node src/image_converter.cpp)
ament_target_dependencies(image_converter_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
add_executable(mode_selector_node src/mode_select.cpp)


ament_target_dependencies(mode_selector_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)



target_link_libraries(image_converter_node ${cpp_typesupport_target})
target_link_libraries(mode_selector_node ${cpp_typesupport_target})




install(TARGETS
  image_converter_node
  mode_selector_node
 
  
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

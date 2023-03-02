cmake_minimum_required(VERSION 3.8)
project(large_msg_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
        ${large_msg_generator_node_INCLUDE_DIRS}
        SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

add_executable(large_msg_generator_node src/large_msg_generator_node.cpp)
ament_target_dependencies(large_msg_generator_node
        rclcpp
        sensor_msgs
        )
target_link_libraries(large_msg_generator_node "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
install(TARGETS large_msg_generator_node
        DESTINATION lib/${PROJECT_NAME}
        )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()

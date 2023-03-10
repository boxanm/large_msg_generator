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

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker
        rclcpp
        sensor_msgs
        )


add_executable(listener src/listener.cpp)
ament_target_dependencies(listener
        rclcpp
        sensor_msgs
        )

target_link_libraries(talker "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)
install(TARGETS talker listener
        DESTINATION lib/${PROJECT_NAME}
        )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()

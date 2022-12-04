cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ChangeMessage.srv")

add_executable(talker src/publisher_member_function.cpp include/beginner_tutorials/publisher_function.hpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros rosbag2_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)



add_executable(listener src/subscriber_member_function.cpp include/beginner_tutorials/subscriber_function.hpp)
ament_target_dependencies(listener rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(listener "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)

# add_executable(client src/client_node.cpp)
# ament_target_dependencies(client rclcpp std_msgs)

add_executable(minimal_param_node src/publisher_parameter_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp std_msgs)

install(TARGETS
  talker
  listener
  minimal_param_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ## Lines to skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ## Run linters on all package contects except those ones that are listed here
  ament_lint_auto_find_test_dependencies()

  ## gtest executables
  set(TESTFILES
    test/main.cpp
    test/test_talker.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)


  # run test if build passes
  install(TARGETS
  test_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

endif()



ament_package()

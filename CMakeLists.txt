cmake_minimum_required(VERSION 3.8)
project(mullet_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DisableOffboard.action"
  "action/EnableOffboard.action"
  "action/Land.action"
  "action/Takeoff.action"
)

set(MULLET_CORE_NODE_SOURCES
  src/action_lifecycle_coordinator.cpp
  src/drone_state_machine_actions.cpp
  src/drone_state_machine_commands.cpp
  src/drone_state_machine.cpp
  src/drone_state_machine_helpers.cpp
  src/drone_state_machine_predicates.cpp
  src/drone_state_machine_states.cpp
  src/drone_state_machine_timing.cpp
  src/px4_interface.cpp
)

add_executable(${PROJECT_NAME}_node
  src/drone_start.cpp
  ${MULLET_CORE_NODE_SOURCES}
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  px4_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  function(mullet_core_configure_gtest target_name)
    target_compile_features(${target_name} PUBLIC cxx_std_17)
    target_include_directories(${target_name} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    )
  endfunction()

  ament_add_gtest(test_vehicle_instance
    test/test_vehicle_instance.cpp
  )
  mullet_core_configure_gtest(test_vehicle_instance)

  ament_add_gtest(test_state_machine_helpers
    test/test_state_machine_helpers.cpp
    src/drone_state_machine_helpers.cpp
  )
  mullet_core_configure_gtest(test_state_machine_helpers)
  ament_target_dependencies(test_state_machine_helpers
    rclcpp
  )

  ament_add_gtest(test_drone_state_machine_behavior
    test/test_drone_state_machine_behavior.cpp
    ${MULLET_CORE_NODE_SOURCES}
  )
  mullet_core_configure_gtest(test_drone_state_machine_behavior)
  ament_target_dependencies(test_drone_state_machine_behavior
    rclcpp
    rclcpp_action
    px4_msgs
    std_msgs
  )
  target_link_libraries(test_drone_state_machine_behavior ${cpp_typesupport_target})

  ament_add_pytest_test(test_mullet_core_launch
    test/test_mullet_core_launch.py
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()

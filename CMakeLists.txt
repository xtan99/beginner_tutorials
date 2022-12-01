cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PrintNewString.srv"
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)



rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(rclcpp REQUIRED)
  #set(ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  function(custom_executable target)
    add_executable(${target} ${ARGN})
    target_compile_definitions(${target}
      PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(${target} "${cpp_typesupport_target}")
    ament_target_dependencies(${target}
      "rclcpp")
  endfunction()

  function(custom_gtest_executable target)
    ament_add_gtest_executable(${target} ${ARGN})
    target_compile_definitions(${target}
      PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(${target} "${cpp_typesupport_target}")
    ament_target_dependencies(${target}
      "rclcpp")
  endfunction()

  macro(custom_launch_test_two_executables test_name executable1 executable2)
    cmake_parse_arguments(_ARG "" "ARGS1;ARGS2;RMW1;RMW2" "" ${ARGN})
    set(TEST_NAME "${test_name}")
    set(TEST_EXECUTABLE1 "$<TARGET_FILE:${executable1}>")
    set(TEST_EXECUTABLE1_ARGS "${_ARG_ARGS1}")
    set(TEST_EXECUTABLE1_NAME "${executable1}")
    set(TEST_RMW_IMPLEMENTATION1 "${_ARG_RMW1}")
    set(TEST_EXECUTABLE2 "$<TARGET_FILE:${executable2}>")
    set(TEST_EXECUTABLE2_ARGS "${_ARG_ARGS2}")
    set(TEST_EXECUTABLE2_NAME "${executable2}")
    set(TEST_RMW_IMPLEMENTATION2 "${_ARG_RMW2}")
    configure_file(
      test/test_two_executables.py.in
      ${test_name}${target_suffix}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET "${test_name}${target_suffix}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ${_ARG_UNPARSED_ARGUMENTS}
    )
    if(TEST ${test_name}${target_suffix})
      set_tests_properties(${test_name}${target_suffix}
        PROPERTIES DEPENDS "${executable1}${target_suffix} ${executable2}${target_suffix}"
      )
    endif()
  endmacro()

  macro(targets)

    # Service tests single implementation
    custom_launch_test_two_executables(test_services_cpp
      test_services_server_cpp test_services_client_cpp
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation}
      TIMEOUT 60)

  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")

  call_for_each_rmw_implementation(targets)

  # Service tests single implementation
  custom_executable(test_services_server_cpp "test/test_services_server.cpp")
  custom_gtest_executable(test_services_client_cpp "test/test_services_client.cpp")

endif()  # BUILD_TESTING


target_link_libraries(talker "${cpp_typesupport_target}")

install(TARGETS
  talker
  listener

  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

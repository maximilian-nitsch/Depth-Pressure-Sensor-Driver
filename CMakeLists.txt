cmake_minimum_required(VERSION 3.8)
project(external_pressure_sensor_package)

# Set default value for INCLUDE_HARDWARE_TEST
option(INCLUDE_HARDWARE_TEST "Include hardware test" FALSE)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# add keller library
include_directories(include/endianness)

set(KELLER_HEADER_FILES
  include/depth_pressure_sensor_driver_package/keller_sensor_driver.h
  include/depth_pressure_sensor_driver_package/keller_serial_interface_emulator.h
  include/depth_pressure_sensor_driver_package/keller_serial_interface.h
  include/depth_pressure_sensor_driver_package/serial_interface.h)

add_library(keller
  src/keller_sensor_driver.cpp
  src/keller_serial_interface_emulator.cpp
  src/keller_serial_interface.cpp
  src/serial_interface.cpp
  ${KELLER_HEADER_FILES})

target_include_directories(keller PUBLIC include/depth_pressure_sensor_driver_package)

add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC keller)

add_executable(${PROJECT_NAME}_node src/depth_pressure_sensor_driver_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  tf2
  tf2_ros)

target_link_libraries(${PROJECT_NAME}_node keller)

install(TARGETS keller DESTINATION lib/${PROJECT_NAME})
install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/depth_pressure_sensor_driver_package launch config
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS keller
  EXPORT keller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips uncrustify (source code formatting)
  # set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)

  # core unit testing suite, emulates the sensor
  set(TEST_FILES
    test/main.cpp
    test/depth_pressure_sensor_driver_test.h
    test/depth_pressure_sensor_driver_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})

  target_link_libraries(${PROJECT_NAME}_test keller)

  install(TARGETS
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})

  if(INCLUDE_HARDWARE_TEST)
    # same tests for the real connected sensor
    set(TEST_FILES_HARDWARE
      test/main.cpp
      test/depth_pressure_sensor_driver_test.h
      test/depth_pressure_sensor_driver_test_hardware.cpp)

    ament_add_gtest(${PROJECT_NAME}_test_hardware ${TEST_FILES_HARDWARE})

    target_link_libraries(${PROJECT_NAME}_test_hardware keller)

    install(TARGETS
      ${PROJECT_NAME}_test_hardware
      DESTINATION lib/${PROJECT_NAME})
  endif()


  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify)

  # enforce linters and static code analyzers defined in ament_lint_common package
  ament_lint_auto_find_test_dependencies()

  # uncomment to include uncrustify explicitly
  # find_package(ament_cmake_uncrustify)
  # ament_uncrustify(CONFIG_FILE "./uncrustify.cfg" TESTNAME "custom_uncrustify")

endif()

ament_package()

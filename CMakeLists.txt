cmake_minimum_required(VERSION 3.5)
project(robot_config)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color=always)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cti_msgs REQUIRED)
find_package(ctilog REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/robot_config.cpp
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${YAML_CPP_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}_node
  ${YAML_CPP_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_node
  "ament_index_cpp"
  "cti_msgs"
  "ctilog"
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "diagnostic_msgs"
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

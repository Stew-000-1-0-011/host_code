cmake_minimum_required(VERSION 3.8)
project(chibarobo25_main)

set(CMAKE_CXX_COMPILER clang++-20)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(main_node
  src/main_node.cpp
)
ament_target_dependencies(main_node
    rclcpp
    geometry_msgs
)
target_link_libraries(main_node
  ${PROJECT_SOURCE_DIR}/../../install/lib/host_code/libstatic_lib.a
)
target_include_directories(main_node
  PRIVATE
  ../../include/
)

install(
  TARGETS main_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

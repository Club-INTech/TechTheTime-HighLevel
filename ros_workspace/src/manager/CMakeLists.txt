cmake_minimum_required(VERSION 3.5)
project(manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msg_srv REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(ActionOrderDirectory ../srvs_msgs/action_msg_srv_shared)

set(ActionOrderHeaders ${ActionOrderDirectory}/order_codes.hpp ${ActionOrderDirectory}/order_binder.hpp)

add_executable(manager src/manager.cpp ${ActionOrderDirectory})
ament_target_dependencies(manager
  rclcpp action_msg_srv)

target_include_directories(manager PUBLIC ${ActionOrderDirectory})

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

install(TARGETS
  manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()

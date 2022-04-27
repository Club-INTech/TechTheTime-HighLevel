cmake_minimum_required(VERSION 3.5)
project(manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall)

include(FetchContent)
message("Fetching yaml-cpp")
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_SHALLOW ON
  GIT_TAG yaml-cpp-0.6.3
)

FetchContent_GetProperties(yaml-cpp)
if(NOT yaml-cpp_POPULATED)
	message(STATUS "Populating yaml-cpp...")
	FetchContent_Populate(yaml-cpp)
	# Add here options for yaml-cpp building
	set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "disable yaml tests")
	add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
	message(STATUS "Done.")
endif()

include_directories(BEFORE SYSTEM ${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msg_srv REQUIRED)
find_package(motion_msg_srv REQUIRED)
# find_package(example_topic REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(SharedDirectory ../shared)

include_directories(${SharedDirectory})

add_executable(${PROJECT_NAME} 
  src/manager.cpp 
  src/subscriber/MotionSubscriber.cpp
  src/dev/order_reader.cpp
  src/script/script.cpp
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

ament_target_dependencies(manager
  rclcpp 
  action_msg_srv
  motion_msg_srv
)


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

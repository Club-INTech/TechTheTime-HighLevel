cmake_minimum_required(VERSION 3.5)

#including fetch content
include(FetchContent)

project(microcontroller_proxy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Fetches expected nedded for k2o functionning
message("Fetching Expected...")
FetchContent_Declare(
  Expected
  GIT_REPOSITORY https://github.com/TartanLlama/expected.git
  GIT_TAG v1.0.0
)

# Making available expected
FetchContent_MakeAvailable(Expected)

#Fetching yaml-cpp to process yaml config for microcontroller_proxy launching
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

#Add compile flag to show warnings and errors
add_compile_options(-Wall)


# Find dependencies
# Ament and rclcpp for ros2 functionning
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Ros custom interfaces
find_package(action_msg_srv REQUIRED)
find_package(motion_msg_srv REQUIRED)

# Ros std interface
find_package(std_msgs REQUIRED)


# Include submodules' headers

set(KeyToOrderDirectory ./lib/Key-To-Order/include)
set(UnpaddedDirectory ./lib/Unpadded/include)
set(MP11Directory ./lib/mp11/include)
set(ConfigDirectory ./lib/config/include)
set(StaticAssertDirectory ./lib/static_assert/include)
set(TypeTraitsDirectory ./lib/type_traits/include)

include_directories(${UnpaddedDirectory})
include_directories(${MP11Directory})
include_directories(${StaticAssertDirectory})
include_directories(${TypeTraitsDirectory})
include_directories(${ConfigDirectory})
include_directories(${KeyToOrderDirectory})

# Include path

set(SharedDirectory ../shared)
set(IncludePath ./include)
set(TechTheTimeSharedDirectory ./include/TechTheTime-Shared)

include_directories(${TechTheTimeSharedDirectory})
include_directories(${SharedDirectory})
include_directories(${IncludePath})

# Compile sources and produce executable
add_executable(
  ${PROJECT_NAME} 
  src/com/SerialPort.cpp
  src/ros/cli_serv/ActionService.cpp 
  src/ros/pub_sub/MotionPublisher.cpp
  src/ros/pub_sub/AlertSubscriber.cpp
  src/microcontroller_proxy.cpp
)

# Linking expected
target_link_libraries(${PROJECT_NAME} expected yaml-cpp)

# Linking ament dependencies for ROS2
ament_target_dependencies(${PROJECT_NAME}
  rclcpp action_msg_srv motion_msg_srv std_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()

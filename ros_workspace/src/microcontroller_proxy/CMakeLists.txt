cmake_minimum_required(VERSION 3.5)

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

message("Fetching Expected...")
FetchContent_Declare(
  Expected
  GIT_REPOSITORY https://github.com/TartanLlama/expected.git
  GIT_TAG v1.0.0
)

FetchContent_MakeAvailable(Expected)

add_compile_options(-Wall -Wextra)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msg_srv REQUIRED)
find_package(motion_msg_srv REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(ActionOrderDirectory ../srvs_msgs/action_msg_srv_shared)
set(KeyToOrderDirectory ./lib/Key-To-Order/include)
set(SharedDirectory ./src/TechTheTime-Shared)

set(UnpaddedDirectory ./lib/Unpadded/include)
set(MP11Directory ./lib/mp11/include)
set(ConfigDirectory ./lib/config/include)
set(StaticAssertDirectory ./lib/static_assert/include)
set(TypeTraitsDirectory ./lib/type_traits/include)

set(ActionOrderHeaders ${ActionOrderDirectory}/order_codes.hpp ${ActionOrderDirectory}/order_binder.hpp)

include_directories(${KeyToOrderDirectory})
include_directories(${SharedDirectory})
include_directories(${UnpaddedDirectory})
include_directories(${MP11Directory})
include_directories(${StaticAssertDirectory})
include_directories(${TypeTraitsDirectory})

add_executable(
  ${PROJECT_NAME} 
  src/serial/SerialPort.cpp
  src/service/ActionService.cpp 
  src/microcontroller_proxy.cpp
  src/publisher/MotionPublisher.cpp 
  ${ActionOrderHeaders} 
)
target_include_directories(${PROJECT_NAME} PUBLIC ${ActionOrderDirectory})
target_link_libraries(${PROJECT_NAME} expected)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp action_msg_srv motion_msg_srv)

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
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()

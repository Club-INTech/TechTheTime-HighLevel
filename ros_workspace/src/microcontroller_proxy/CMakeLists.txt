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

message("Fetching google test...")
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_compile_options(-Wall -Wextra)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_msg_srv REQUIRED)
find_package(motion_msg_srv REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(SharedDirectory ../shared)
set(KeyToOrderDirectory ./lib/Key-To-Order/include)
set(TechTheTimeSharedDirectory ./src/TechTheTime-Shared)

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
include_directories(${TechTheTimeSharedDirectory})

include_directories(${SharedDirectory})

add_executable(
  ${PROJECT_NAME} 
  src/serial/SerialPort.cpp
  src/service/ActionService.cpp 
  src/microcontroller_proxy.cpp
  src/publisher/MotionPublisher.cpp
  src/alert/AlertSubscriber.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${SharedDirectory})
target_link_libraries(${PROJECT_NAME} expected)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp action_msg_srv motion_msg_srv std_msgs)

enable_testing()

add_executable(
  microcontroller_proxy_test
  test/main_test.cpp
)

target_link_libraries(
  microcontroller_proxy_test
  gtest_main
)

include(GoogleTest)
gtest_discover_tests(microcontroller_proxy_test)

install(TARGETS
  ${PROJECT_NAME}
  microcontroller_proxy_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()

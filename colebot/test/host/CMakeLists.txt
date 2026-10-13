# Host tests for the hardware-free firmware code (PLAN.md §7.9).
# Plain CMake, no ESP-IDF: builds interfaces + board + core from the same component
# directories the firmware uses. An IDF include in core fails this build, which
# enforces the dependency rule in PLAN.md §3.4.
#
#   cmake -S colebot/test/host -B colebot/test/host/build
#   cmake --build colebot/test/host/build
#   ctest --test-dir colebot/test/host/build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(colebot_host_tests CXX)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)  # extracted files get the extraction time (CMake >= 3.24)
endif()
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
  URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # MSVC: match the CRT of the tests
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(COMPONENTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../components)
add_subdirectory(${COMPONENTS_DIR}/interfaces interfaces)
add_subdirectory(${COMPONENTS_DIR}/board board)
add_subdirectory(${COMPONENTS_DIR}/core core)

add_executable(colebot_host_tests
  test_connection_manager.cpp
  test_drive_controller.cpp
  test_drive_math.cpp
  test_mock_esc.cpp
  test_motor.cpp
  test_throttle_3d.cpp
  test_wheel_command_handler.cpp
)
target_include_directories(colebot_host_tests PRIVATE doubles)
target_link_libraries(colebot_host_tests PRIVATE colebot_core GTest::gmock_main)

include(GoogleTest)
gtest_discover_tests(colebot_host_tests)

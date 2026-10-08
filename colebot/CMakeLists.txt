# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# The firmware brings up WiFi and Ethernet itself (PLAN.md §5.1); only micro-ROS's UDP transport is used.
set(BUILD_WLAN_INTERFACE OFF CACHE BOOL "micro-ROS component's own WiFi bring-up" FORCE)
set(BUILD_ETHERNET_INTERFACE OFF CACHE BOOL "micro-ROS component's own Ethernet bring-up" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(colebot)

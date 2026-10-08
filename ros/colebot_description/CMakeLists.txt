cmake_minimum_required(VERSION 3.16)
project(colebot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Source files carry an SPDX GPL-3.0-or-later tag (PLAN.md §10); ament_copyright
  # only accepts the full license block, so it is skipped.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

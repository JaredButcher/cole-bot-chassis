# Header-only pure abstract interfaces. No ESP-IDF or micro-ROS headers (PLAN.md §3.3),
# so the same headers build on the target and in the host tests.
if(DEFINED ESP_PLATFORM)
  idf_component_register(INCLUDE_DIRS "include")
else()
  add_library(colebot_interfaces INTERFACE)
  target_include_directories(colebot_interfaces INTERFACE include)
endif()

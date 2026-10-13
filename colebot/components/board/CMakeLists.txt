# Header-only: pin map and board constants. No ESP-IDF dependencies.
if(DEFINED ESP_PLATFORM)
  idf_component_register(INCLUDE_DIRS "include")
else()
  add_library(colebot_board INTERFACE)
  target_include_directories(colebot_board INTERFACE include)
endif()

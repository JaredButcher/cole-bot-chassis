# Hardware-free logic. Depends only on interfaces, board and the C++ standard library
# (PLAN.md §3.4). The host test build compiles these same sources without ESP-IDF,
# so an IDF include here fails the host build.
set(COLEBOT_CORE_SRCS
  "src/throttle_3d.cpp"
)

if(DEFINED ESP_PLATFORM)
  idf_component_register(
    SRCS ${COLEBOT_CORE_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES interfaces board)
else()
  add_library(colebot_core STATIC ${COLEBOT_CORE_SRCS})
  target_include_directories(colebot_core PUBLIC include)
  target_link_libraries(colebot_core PUBLIC colebot_interfaces colebot_board)
endif()

# ESP-IDF and AlfredoDShot adapters for the core interfaces. Verified on the robot,
# not in host tests. Nothing depends on this component except main (PLAN.md §3.4).
idf_component_register(
  SRCS "src/alfredo_esc.cpp" "src/esp_clock.cpp"
  INCLUDE_DIRS "include"
  REQUIRES interfaces core board
  PRIV_REQUIRES alfredo_dshot esp_timer)

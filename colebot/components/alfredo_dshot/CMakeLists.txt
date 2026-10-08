# AlfredoDShot (upstream submodule, never edited) built as a plain ESP-IDF component.
# See docs/plans/alfredo-dshot.md §2.
idf_component_register(
  SRCS         "AlfredoDShot/src/AlfredoDShot.cpp"
  INCLUDE_DIRS "AlfredoDShot/src" "compat"
  REQUIRES     esp_driver_rmt esp_driver_gpio esp_timer
  PRIV_REQUIRES hal soc freertos)
# Third-party code: don't let its warnings fail our build.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)

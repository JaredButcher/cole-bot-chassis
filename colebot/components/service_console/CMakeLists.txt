# USB service console (PLAN.md §7.8): esp_console glue over core ConsoleCommands.
idf_component_register(
  SRCS "src/service_console.cpp"
  INCLUDE_DIRS "include"
  REQUIRES core interfaces
  PRIV_REQUIRES console)

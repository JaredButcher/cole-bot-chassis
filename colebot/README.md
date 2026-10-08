# colebot firmware

ESP32-C6 chassis controller: a micro-ROS node that drives the two wheels as ros2_control joints. Design and roadmap: [`PLAN.md`](../PLAN.md).

## Build

ESP-IDF **v5.5.5**. Either open this folder in the devcontainer (`.devcontainer/`), or use a local install with the micro-ROS build dependencies added to the IDF Python environment:

```bash
. $IDF_PATH/export.sh
pip install catkin_pkg colcon-common-extensions lark
```

Build in a shell where ROS is **not** sourced. The first build also builds `libmicroros`, which clones the micro-ROS sources and takes several minutes.

```bash
git submodule update --init      # AlfredoDShot, micro_ros_espidf_component
idf.py build                     # target esp32c6 comes from sdkconfig.defaults
idf.py -p <port> flash monitor   # native USB port (USB-Serial-JTAG)
```

## Host tests

Hardware-free code in `components/core` is tested on a PC with plain CMake and GoogleTest; no ESP-IDF needed.

```bash
cmake -S test/host -B test/host/build
cmake --build test/host/build
ctest --test-dir test/host/build --output-on-failure
```

## Layout

| Path | Contents |
| ---- | -------- |
| `main/app_main.cpp` | Composition root: creates objects, wires them, starts tasks |
| `components/board` | Pin map, drive defaults, ROS names (header-only, no IDF) |
| `components/interfaces` | Pure abstract interfaces (`IClock`, `IEsc`, …), no IDF |
| `components/core` | Hardware-free logic, host-tested |
| `components/platform` | ESP-IDF / AlfredoDShot / micro-ROS adapters |
| `components/service_console` | USB service console (`esp_console`) |
| `components/alfredo_dshot` | AlfredoDShot submodule + IDF wrapper and Arduino shim |
| `components/micro_ros_espidf_component` | micro-ROS submodule (`jazzy` branch) |
| `app-colcon.meta` | micro-ROS static entity limits |
| `test/host` | Host tests and test doubles |

# colebot host packages (ROS 2 Jazzy)

Host side of the Cole bot chassis: the firmware in `colebot/` is a micro-ROS node, and these packages run `ros2_control` around it. Design: [`PLAN.md`](../PLAN.md) §4 and §7.6.

| Package | Contents |
| ------- | -------- |
| `colebot_description` | URDF/xacro with the `ros2_control` tag (firmware topics, or mock hardware) |
| `colebot_bringup` | Launch file and config: `robot_state_publisher`, `ros2_control_node`, `diff_drive_controller`, `twist_mux` |

## Environment

Open this folder in the devcontainer (`.devcontainer/`, based on `ros:jazzy`), or install ROS 2 Jazzy on Ubuntu 24.04 and run `rosdep install --from-paths . --ignore-src -y`.

## Build and test

```bash
colcon build --symlink-install
colcon test && colcon test-result --verbose
source install/setup.bash
```

## Run

Without the robot (mock hardware):

```bash
ros2 launch colebot_bringup bringup.launch.py use_mock_hardware:=true
```

With the robot: start the micro-ROS agent, then the bringup.

```bash
docker compose -f agent/compose.yaml up -d
ros2 launch colebot_bringup bringup.launch.py              # control_rate:=50 by default
```

Drive with `teleop_twist_keyboard` (stamped, into the `teleop` input of `twist_mux`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel/teleop
```

`twist_mux` inputs: `/cmd_vel/teleop` (priority 100), `/cmd_vel/cli` (90), `/cmd_vel/nav` (10). All inputs are blocked while `/chassis/estop_state` is true.

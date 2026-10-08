# SPDX-License-Identifier: GPL-3.0-or-later
"""Launch test: the host stack runs on mock hardware and drives from cmd_vel (PLAN.md §6, §7.9)."""

import time
import unittest

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from std_msgs.msg import Bool

CONTROL_RATE = 25  # not the default, to check the launch argument reaches the controllers


@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('colebot_bringup'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'use_mock_hardware': 'true',
            'control_rate': str(CONTROL_RATE),
        }.items())
    return LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestMockBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_mock_bringup')
        cls.odom = None
        cls.node.create_subscription(
            Odometry, '/diff_drive_controller/odom', cls._on_odom, 10)
        cls.cmd_pub = cls.node.create_publisher(TwistStamped, '/cmd_vel/teleop', 10)
        cls.estop_pub = cls.node.create_publisher(Bool, '/chassis/estop_state', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_odom(cls, msg):
        cls.odom = msg

    def _spin_for(self, seconds, publish_cmd=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish_cmd is not None:
                msg = TwistStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.twist.linear.x = publish_cmd
                self.cmd_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _call(self, client, request, timeout=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), client.srv_name)
        return future.result()

    def _wait_for_controllers(self, timeout=60.0):
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        end = time.monotonic() + timeout
        states = {}
        while time.monotonic() < end:
            result = self._call(client, ListControllers.Request())
            states = {c.name: c.state for c in result.controller}
            if (states.get('diff_drive_controller') == 'active'
                    and states.get('joint_state_broadcaster') == 'active'
                    and states.get('wheel_velocity_controller') == 'inactive'):
                return states
            self._spin_for(0.5)
        self.fail(f'controllers not ready: {states}')

    def _odom_x(self):
        self.assertIsNotNone(self.odom, 'no odometry received')
        return self.odom.pose.pose.position.x

    def test_1_controllers_loaded(self):
        self._wait_for_controllers()

    def test_2_control_rate_reaches_controllers(self):
        self._wait_for_controllers()
        client = self.node.create_client(GetParameters, '/diff_drive_controller/get_parameters')
        result = self._call(client, GetParameters.Request(names=['publish_rate']))
        self.assertAlmostEqual(result.values[0].double_value, float(CONTROL_RATE))

        client = self.node.create_client(GetParameters, '/controller_manager/get_parameters')
        result = self._call(client, GetParameters.Request(names=['update_rate']))
        self.assertEqual(result.values[0].integer_value, CONTROL_RATE)

    def test_3_cmd_vel_moves_odometry(self):
        self._wait_for_controllers()
        self._spin_for(1.0)
        start = self._odom_x()
        self._spin_for(2.0, publish_cmd=0.3)
        self.assertGreater(self._odom_x() - start, 0.2)

    def test_4_estop_lock_blocks_cmd_vel(self):
        self._wait_for_controllers()
        latched = Bool(data=True)
        for _ in range(5):
            self.estop_pub.publish(latched)
            self._spin_for(0.1)
        self._spin_for(1.0)  # let any earlier motion and cmd_vel timeouts settle
        start = self._odom_x()
        self._spin_for(2.0, publish_cmd=0.3)
        self.assertLess(abs(self._odom_x() - start), 0.01)

        released = Bool(data=False)
        for _ in range(5):
            self.estop_pub.publish(released)
            self._spin_for(0.1)
        start = self._odom_x()
        self._spin_for(2.0, publish_cmd=0.3)
        self.assertGreater(self._odom_x() - start, 0.2)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # The spawners exit 0 once their controller is loaded; other nodes are stopped by
        # the test harness (SIGINT), which ros2 nodes report as -2 or 0.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])

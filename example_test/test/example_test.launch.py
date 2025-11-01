import time as pytime
import unittest

import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import launch_testing
from launch_ros.actions import Node

from test_utils.sim_setup import generate_sim_test_description

DRONE_POSITION = []
# -----------------------------------------------------------------------------
# (1) Modify the scenario config for this test
# -----------------------------------------------------------------------------
def modify_config(config):
    config["drone_position"] = "0.0 5.0 2.0"  # override something
    DRONE_POSITION[:] = [float(x) for x in config["drone_position"].split()]  # capture for later use
    return config


# -----------------------------------------------------------------------------
# (2) Generate test description and launch test nodes
# -----------------------------------------------------------------------------
def generate_test_description():
    """Launch the base simulation and add the demo_nodes_cpp talker."""
    extra_nodes = [
        Node(
            package="demo_nodes_cpp",
            executable="talker",
            name="test_talker",
            output="screen",
        ),
    ]

    return generate_sim_test_description(
        modify_config_fn=modify_config,
        scenario_value="default",
        extra_nodes=extra_nodes,
        delay=5.0,
    )


# -----------------------------------------------------------------------------
# (3) Tests
# -----------------------------------------------------------------------------
class TestTalker(unittest.TestCase):
    """Example: verify the demo talker publishes on /chatter while sim runs."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("talker_test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_node_starts(self):
        # Check if node is visible in ROS graph
        nodes = self.node.get_node_names()
        self.assertIn("talker_test_node", nodes)

    def test_talker_publishes(self):
        """Check that /chatter topic receives at least one String message."""
        msgs = []
        sub = self.node.create_subscription(
            String,
            "/chatter",
            lambda msg: msgs.append(msg),
            qos_profile=qos_profile_sensor_data,
        )

        end_time = pytime.time() + 5.0
        while pytime.time() < end_time and not msgs:
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs), 0, "No messages received from talker!")

# -----------------------------------------------------------------------------
# (4) Test simulation odom matches drone position
# -----------------------------------------------------------------------------
class TestOdomPosition(unittest.TestCase):
    """Check that the /orca/odom topic matches the configured drone position."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("odom_test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_odom_matches_config(self):
        """Ensure the simulated odometry position matches the configured drone position."""
        from nav_msgs.msg import Odometry

        msgs = []
        sub = self.node.create_subscription(
            Odometry,
            "/orca/odom",
            lambda msg: msgs.append(msg),
            qos_profile=qos_profile_sensor_data,
        )

        # Wait up to 10 seconds for odometry messages
        end_time = pytime.time() + 10.0
        while pytime.time() < end_time and not msgs:
            rclpy.spin_once(self.node, timeout_sec=0.2)

        self.node.destroy_subscription(sub)
        self.assertGreater(len(msgs), 0, "No /orca/odom messages received!")

        odom = msgs[-1]
        pos = odom.pose.pose.position

        # Compare with DRONE_POSITION (populated in modify_config)
        dx = abs(pos.x - DRONE_POSITION[0])
        dy = abs(pos.y - DRONE_POSITION[1])
        dz = abs(pos.z - DRONE_POSITION[2])

        tolerance = 0.2  # meters
        self.assertLessEqual(dx, tolerance, f"Odom X differs by {dx:.2f} m")
        self.assertLessEqual(dy, tolerance, f"Odom Y differs by {dy:.2f} m")
        self.assertLessEqual(dz, tolerance, f"Odom Z differs by {dz:.2f} m")


# -----------------------------------------------------------------------------
# (5) Post-shutdown verification
# -----------------------------------------------------------------------------
@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    """Confirm all processes exited cleanly."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

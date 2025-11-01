import os
import time as pytime
import math
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
import tf2_geometry_msgs
import launch_testing
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from test_utils.sim_setup import generate_sim_test_description


DOCK_POS = []


def modify_config(config):
    """Apply test-specific overrides to the simulation config."""
    config["drone_position"] = "0.0 7.5 2.0"
    DOCK_POS[:] = [float(x) for x in config["docking_station_position"].split()]
    return config


def generate_test_description():
    """Launch the simulation and add the ArUco detector node."""

    aruco_config_path = os.path.join(
        get_package_share_directory("aruco_detector"), "config", "aruco_detector_params.yaml"
    )

    extra_nodes = [
        Node(
            package="aruco_detector",
            executable="aruco_detector_node",
            name="aruco_detector_node",
            parameters=[aruco_config_path],
            remappings=[
                ("/image", "/cam_down/image_color"),
                ("/camera_info", "/cam_down/camera_info"),
            ],
            output="screen",
        )
    ]

    return generate_sim_test_description(
        modify_config_fn=modify_config,
        scenario_value="docking",
        extra_nodes=extra_nodes,
        delay=5.0,
    )



class TestArucoDetectorRuntime(unittest.TestCase):
    """Runtime tests for ArUco detector inside simulation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aruco_test_node")

    def tearDown(self):
        self.node.destroy_node()


    def test_node_starts(self):
        """Ensure that the ArUco detector node appears in the ROS graph."""
        end_time = pytime.time() + 5.0
        while pytime.time() < end_time:
            nodes = self.node.get_node_names()
            if "aruco_detector_node" in nodes:
                break
            rclpy.spin_once(self.node, timeout_sec=0.2)
        else:
            self.fail("aruco_detector_node not found in ROS graph")


    def test_publishes_detections(self):
        """Check that /aruco_detector/board publishes PoseStamped messages."""
        msgs = []
        sub = self.node.create_subscription(
            PoseStamped,
            "/aruco_detector/board",
            lambda msg: msgs.append(msg),
            qos_profile=qos_profile_sensor_data,
        )

        try:
            end_time = pytime.time() + 10.0
            while pytime.time() < end_time and not msgs:
                rclpy.spin_once(self.node, timeout_sec=0.2)
            self.assertGreater(len(msgs), 0, "No PoseStamped messages received!")
        finally:
            self.node.destroy_subscription(sub)


    def test_pose_transformed_to_odom_close_to_dock(self):
        """Transform the detection pose into odom and verify proximity to dock."""
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self.node)

        msgs = []

        def callback(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(
            PoseStamped, "/aruco_detector/board", callback, qos_profile=qos_profile_sensor_data
        )

        try:
            success = False
            end_time = pytime.time() + 10.0
            last_info = None

            while pytime.time() < end_time and not success:
                rclpy.spin_once(self.node, timeout_sec=0.2)

                for pose_msg in list(msgs):
                    try:
                        trans = tf_buffer.lookup_transform(
                            "odom",
                            pose_msg.header.frame_id,
                            pose_msg.header.stamp,
                            timeout=rclpy.duration.Duration(seconds=0.1),
                        )

                        pose_odom = tf2_geometry_msgs.do_transform_pose(pose_msg.pose, trans)
                        pos = pose_odom.position
                        dock = DOCK_POS
                        dist = math.sqrt(
                            (pos.x - dock[0]) ** 2
                            + (pos.y - dock[1]) ** 2
                            + (pos.z - dock[2]) ** 2
                        )

                        if dist <= 0.3:
                            success = True
                            break
                        else:
                            last_info = f"Got transform but dist={dist:.2f}m"
                    except tf2_ros.TransformException as ex:
                        last_info = str(ex)
                        continue

            self.assertTrue(success, f"No valid transform within 10s. Last info: {last_info}")

        finally:
            self.node.destroy_subscription(sub)
            # Note: do not destroy tf_listener manually
            pass


@launch_testing.post_shutdown_test()
class TestArucoAfterShutdown(unittest.TestCase):
    """Verify processes exit cleanly after shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

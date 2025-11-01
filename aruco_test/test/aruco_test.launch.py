import os
import yaml
import tempfile
import time as pytime
import unittest

import rclpy
import geometry_msgs.msg as geometry_msgs
from rclpy.qos import qos_profile_sensor_data
import math
import tf2_ros
import tf2_geometry_msgs

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import launch_testing.actions
import launch_testing
import launch

DOCK_POS = []

def load_scenario_config(scenario_val):
    """
    Loads the scenario config YAML for the given scenario.
    """
    config_path = os.path.join(
        get_package_share_directory("stonefish_sim"),
        "config",
        f"{scenario_val}_config.yaml"
    )

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Missing scenario config: {config_path}")
    
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

def modify_scenario_config(config):
    """
    Applies test-specific overrides to the scenario config.
    """
    config["drone_position"] = "0.0 7.5 2.0"
    DOCK_POS[:] = [float(x) for x in config["docking_station_position"].split()]
    return config

def write_temp_config(config):
    tmp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump(config, tmp_file)
    tmp_file.close()
    return tmp_file.name


def launch_setup(context, *args, **kwargs):
    scenario_val = LaunchConfiguration("scenario").perform(context)

    scnenario_config = load_scenario_config(scenario_val)

    test_config = modify_scenario_config(scnenario_config)

    tmp_file_path = write_temp_config(test_config)

    sim_launch_file = os.path.join(
        get_package_share_directory("stonefish_sim"),
        "launch",
        "simulation.launch.py"
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file),
        launch_arguments={
            "scenario": scenario_val,
            "scenario_config_override": tmp_file_path,
        }.items(),
    )

    sim_interface_node = Node(
                package="vortex_sim_interface",
                executable="vortex_sim_interface",
                name="vortex_sim_interface",
                output="screen",
                emulate_tty=True,
            )

    aruco_config_path = os.path.join(
        get_package_share_directory("aruco_detector"), "config", "aruco_detector_params.yaml"
    )

    aruco_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        parameters=[aruco_config_path],
        remappings=[
            ('/image', '/cam_down/image_color'),
            ('/camera_info', '/cam_down/camera_info'),
        ],
        output="screen",
    )

    return [sim_launch, aruco_node, sim_interface_node]


def generate_test_description():
    stonefish_dir = get_package_share_directory("stonefish_sim")

    sim_scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="docking",
        description="scenario to run the simulation for"
    )

    sim_scenario_config_override = DeclareLaunchArgument(
        "scenario_config_override",
        default_value="",
        description="Path to override scenario config YAML"
    )

    return LaunchDescription([
        sim_scenario_arg,
        sim_scenario_config_override,
        OpaqueFunction(function=launch_setup),
        TimerAction(
            period=5.0,
            actions=[launch_testing.actions.ReadyToTest()],
        )
    ]), {}


class TestArucoDetectorRuntime(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('aruco_detector_node')
    
    def tearDown(self):
        self.node.destroy_node()

    def test_node_starts(self):
        # Check if node is visible in ROS graph
        nodes = self.node.get_node_names()
        self.assertIn("aruco_detector_node", nodes)

    def test_publishes_detections(self):
        """
        Subscribes to the aruco_detector output topic and waits for a message.
        Replace '/aruco_detections' with the actual topic name from your package.
        """
        msgs_rx = []
        sub = self.node.create_subscription(
            geometry_msgs.PoseStamped, '/aruco_detector/board',
            lambda msg: msgs_rx.append(msg), qos_profile=qos_profile_sensor_data)
        try:
            # Listen to the pose topic for 10 s
            end_time = pytime.time() + 10
            while pytime.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 1 message received
            assert len(msgs_rx) > 1
        finally:
            self.node.destroy_subscription(sub)

    def test_pose_transformed_to_odom_close_to_dock(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self.node)

        msgs_rx = []

        def callback(msg):
            msgs_rx.append(msg)

        sub = self.node.create_subscription(
            geometry_msgs.PoseStamped,
            '/aruco_detector/board',
            callback,
            qos_profile=qos_profile_sensor_data
        )

        try:
            success = False
            end_time = pytime.time() + 10.0  # poll window
            last_info = None  # store last error for debugging

            while pytime.time() < end_time and not success:
                # process any new messages
                rclpy.spin_once(self.node, timeout_sec=0.2)

                # iterate over all received messages so far
                for pose_msg in list(msgs_rx):
                    try:
                        trans = tf_buffer.lookup_transform(
                            'odom',
                            pose_msg.header.frame_id,
                            pose_msg.header.stamp,
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )

                        pose_odom = tf2_geometry_msgs.do_transform_pose(pose_msg.pose, trans)
                        pos = pose_odom.position
                        dock = DOCK_POS
                        dist = math.sqrt(
                            (pos.x - dock[0]) ** 2 +
                            (pos.y - dock[1]) ** 2 +
                            (pos.z - dock[2]) ** 2
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
                # Do NOT call tf_listener.destroy()!



@launch_testing.post_shutdown_test()
class TestArucoDetectorAfterShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Ensure all processes finished cleanly
        launch_testing.asserts.assertExitCodes(proc_info)

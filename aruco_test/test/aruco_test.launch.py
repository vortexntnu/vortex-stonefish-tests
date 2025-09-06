import os
import yaml
import tempfile
import time as pytime
import unittest

import rclpy
import geometry_msgs.msg as geometry_msgs
from rclpy.qos import qos_profile_sensor_data

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import launch_testing.actions
import launch_testing
import launch

def load_scenario_config(task_val):
    """
    Loads the scenario config YAML for the given task.
    """
    config_path = os.path.join(
        get_package_share_directory("stonefish_sim"),
        "config",
        f"{task_val}_config.yaml"
    )

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Missing scenario config: {config_path}")
    
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

def modify_scenario_config(config):
    """
    Applies test-specific overrides to the scenario config.
    """
    config["drone_position"] = "0.0 7.5 0.0"
    return config

def write_temp_config(config):
    tmp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump(config, tmp_file)
    tmp_file.close()
    return tmp_file.name


def launch_setup(context, *args, **kwargs):
    task_val = LaunchConfiguration("task").perform(context)

    scnenario_config = load_scenario_config(task_val)

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
            "task": task_val,
            "scenario_config_override": tmp_file_path,
        }.items(),
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

    return [sim_launch, aruco_node]


def generate_test_description():
    stonefish_dir = get_package_share_directory("stonefish_sim")

    sim_task_arg = DeclareLaunchArgument(
        "task",
        default_value="docking",
        description="Task to run the simulation for"
    )

    sim_scenario_config_override = DeclareLaunchArgument(
        "scenario_config_override",
        default_value="",
        description="Path to override scenario config YAML"
    )

    return LaunchDescription([
        sim_task_arg,
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


@launch_testing.post_shutdown_test()
class TestArucoDetectorAfterShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Ensure all processes finished cleanly
        launch_testing.asserts.assertExitCodes(proc_info)

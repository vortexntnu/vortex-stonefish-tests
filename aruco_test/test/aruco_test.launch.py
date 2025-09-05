import time as pytime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitution import Substitution
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)

import launch
import launch_ros
import launch_testing.actions
from launch_ros.actions import Node
import os
import yaml

import unittest
import launch_testing.actions
import rclpy
import geometry_msgs.msg as geometry_msgs
from rclpy.qos import QoSProfile, qos_profile_sensor_data

gpu_tasks = [
    "default",
    "docking",
    "pipeline",
    "structure",
    "orca_demo",
    "freya_demo",
    "orca_freya_demo",
]
no_gpu_tasks = [
    "orca_no_gpu",
    "freya_no_gpu",
]


class ConcatenateSubstitutions(Substitution):
    def __init__(self, *substitutions):
        self.substitutions = substitutions

    def perform(self, context):
        return "".join([sub.perform(context) for sub in self.substitutions])


def get_task_and_rendering_value(context):
    """Determine the task and rendering value."""
    rendering_enabled = (
        LaunchConfiguration("rendering").perform(context).lower() == "true"
    )

    task_arg = LaunchConfiguration("task").perform(context)
    if task_arg == "auto":
        task_val = "default" if rendering_enabled else "orca_no_gpu"
    else:
        task_val = task_arg

    if rendering_enabled and task_val not in gpu_tasks:
        raise RuntimeError(
            f"Task '{task_val}' requires GPU rendering to be disabled. "
            f"Choose one of: {', '.join(gpu_tasks)}"
        )
    if not rendering_enabled and task_val not in no_gpu_tasks:
        raise RuntimeError(
            f"Task '{task_val}' requires GPU rendering to be enabled. "
            f"Choose one of: {', '.join(no_gpu_tasks)}"
        )

    return task_val, rendering_enabled


def get_node_details(task_val, rendering_enabled):
    sim_data = LaunchConfiguration("simulation_data")
    sim_rate = LaunchConfiguration("simulation_rate")
    win_x = LaunchConfiguration("window_res_x")
    win_y = LaunchConfiguration("window_res_y")
    rend_qual = LaunchConfiguration("rendering_quality")

    stonefish_dir = get_package_share_directory("stonefish_sim")
    scenario = PathJoinSubstitution(
        [stonefish_dir, "scenarios", TextSubstitution(text=f"{task_val}.scn")]
    )

    if rendering_enabled:
        exe = "stonefish_simulator"
        node_args = [sim_data, scenario, sim_rate, win_x, win_y, rend_qual]
        node_name = "stonefish_simulator"
    else:
        exe = "stonefish_simulator_nogpu"
        node_args = [sim_data, scenario, sim_rate]
        node_name = "stonefish_simulator_nogpu"

    return exe, node_args, node_name


def launch_setup(context, *args, **kwargs):
    task_val, rendering_enabled = get_task_and_rendering_value(context)

    executable, node_args, node_name = get_node_details(task_val, rendering_enabled)

    config_file_path = os.path.join(
        get_package_share_directory("stonefish_sim"),"config",
        task_val + "_config.yaml",
    )

    if not os.path.exists(config_file_path):
        raise FileNotFoundError(f"Configuration file not found: {config_file_path}")

    with open(config_file_path, 'r') as file:
        yaml_params = yaml.safe_load(file)

    test_params = setup_test_params(yaml_params)

    node = Node(
        package="stonefish_ros2",
        executable=executable,
        namespace="stonefish_ros2",
        name=node_name,
        arguments=node_args,
        parameters=[
            test_params
        ],
        output="screen",
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

    return [node, aruco_node]

def setup_test_params(yaml_params):
    yaml_params['drone_position'] = "0.0 7.5 0.0"

    return yaml_params


def generate_test_description():
    stonefish_dir = get_package_share_directory("stonefish_sim")

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "rendering",
                default_value="true",
                description="Enable GPU rendering (true/false)",
            ),
            DeclareLaunchArgument(
                "task",
                default_value="docking",
                description=(
                    "Scenario to load. Use one of "
                    f"{gpu_tasks + no_gpu_tasks}, or leave as 'auto' "
                    "to choose automatically."
                ),
            ),
            DeclareLaunchArgument(
                "simulation_data",
                default_value=PathJoinSubstitution([stonefish_dir, "data"]),
                description="Path to the simulation data folder",
            ),
            DeclareLaunchArgument(
                "simulation_rate",
                default_value="100.0",
                description="Physics update rate [Hz]",
            ),
            DeclareLaunchArgument(
                "window_res_x", default_value="1920", description="Render window width"
            ),
            DeclareLaunchArgument(
                "window_res_y", default_value="1080", description="Render window height"
            ),
            DeclareLaunchArgument(
                "rendering_quality",
                default_value="high",
                description="Rendering quality (high/med/low)",
            ),
            OpaqueFunction(function=launch_setup),
            launch.actions.TimerAction(
                period=5.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]
    )
    return ld, locals()



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

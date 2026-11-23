"""Every node can be constructed with the shipped parameter files.

Needs a sourced ROS 2 workspace (rclpy, aroc_interfaces, ROBOTIS messages);
skipped otherwise, so the pure-Python tests still run in CI without ROS.
Catches parameter typos and declaration bugs before a robot is involved.
"""

import importlib
import os

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('aroc_interfaces.msg')

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
NODES = [
    ('GoNasional.nodes.motion_manager', 'MotionManager'),
    ('GoNasional.nodes.perception_bridge', 'PerceptionBridge'),
    ('GoNasional.nodes.localization_node', 'LocalizationNode'),
    ('GoNasional.nodes.behavior_node', 'BehaviorNode'),
    ('GoNasional.nodes.team_comm', 'TeamComm'),
    ('GoNasional.nodes.game_controller_node', 'GameControllerNode'),
    ('GoNasional.nodes.sim_landmark_observer', 'SimLandmarkObserver'),
]


@pytest.fixture(params=['sim.yaml', 'robot.yaml'])
def ros_args(request):
    args = ['--ros-args', '--params-file', os.path.join(CONFIG, 'soccer.yaml'),
            '--params-file', os.path.join(CONFIG, request.param),
            '-p', 'team_comm.port:=0']
    rclpy.init(args=args)
    yield
    rclpy.shutdown()


@pytest.mark.parametrize('module_name, class_name', NODES)
def test_node_constructs(ros_args, module_name, class_name):
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name, None)
    if cls is None:
        pytest.skip(f'{class_name} not in {module_name}')
    node = cls()
    try:
        assert node.get_name()
    finally:
        sock = getattr(node, 'sock', None)
        if sock is not None:
            sock.close()
        node.destroy_node()

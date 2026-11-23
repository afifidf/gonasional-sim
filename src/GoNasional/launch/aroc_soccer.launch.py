"""AROC soccer stack: motion manager, perception bridge, world model, behavior.

Simulation (after `ros2 launch aroc_sim_bridge bringup.launch.py`):
    ros2 launch GoNasional aroc_soccer.launch.py sim:=true

Robot (after op3_manager, main_vision and trigonocak are running):
    ros2 launch GoNasional aroc_soccer.launch.py

Only this file and the YAML it loads differ between the two; the nodes do not.

Landmarks for localization come from the camera (main_vision boxes ->
perception_bridge) on the robot. In simulation the default is sim_landmarks
(ground truth + noise, cheap); with the sim camera and a YOLO model running
(bringup vision:=yolo) pass landmarks:=vision to use the real pipeline.

robot_state_publisher (op3_description URDF + /robotis/present_joint_states)
gives perception_bridge the camera and feet poses for inverse perspective
mapping. Pass urdf:=false if another launch already publishes them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('GoNasional'), 'config')
    sim = LaunchConfiguration('sim')
    common = os.path.join(config_dir, 'soccer.yaml')
    override = PythonExpression([
        "'", config_dir, "/' + ('sim.yaml' if '", sim, "' == 'true' else 'robot.yaml')"])
    params = [common, override]

    urdf = os.path.join(get_package_share_directory('op3_description'),
                        'urdf', 'robotis_op3.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]),
                                                         value_type=str)}],
        remappings=[('/joint_states', '/robotis/present_joint_states')],
        condition=IfCondition(LaunchConfiguration('urdf')))

    def node(executable, **kwargs):
        return Node(package='GoNasional', executable=executable, output='screen',
                    parameters=params, **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: Webots (aroc_sim_bridge); false: real robot'),
        DeclareLaunchArgument('behavior', default_value='true',
                              description='false: start everything except the behavior node'),
        DeclareLaunchArgument('referee', default_value='true',
                              description='false: do not listen for the GameController'),
        DeclareLaunchArgument('landmarks', default_value='sim',
                              description='sim: ground-truth landmarks in Webots; '
                                          'vision: from YOLO boxes (ignored on the robot)'),
        DeclareLaunchArgument('team', default_value='true',
                              description='false: no team communication (single robot)'),
        DeclareLaunchArgument('urdf', default_value='true',
                              description='false: do not start robot_state_publisher'),
        robot_state_publisher,
        node('motion_manager'),
        node('perception_bridge'),
        node('localization'),
        node('sim_landmarks', condition=IfCondition(PythonExpression([
            "'", sim, "' == 'true' and '", LaunchConfiguration('landmarks'), "' == 'sim'"]))),
        node('game_controller', condition=IfCondition(LaunchConfiguration('referee'))),
        node('team_comm', condition=IfCondition(LaunchConfiguration('team'))),
        node('behavior', condition=IfCondition(LaunchConfiguration('behavior'))),
    ])

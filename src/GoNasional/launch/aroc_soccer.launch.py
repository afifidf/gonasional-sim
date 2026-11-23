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

player:=N, start_pose:=x,y,theta and goalie:=N override soccer.yaml for one
robot; aroc_sim_bridge team.launch.py uses them to run three robots, each in
its own ROS domain.
"""

import copy
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Node names of the stack (super().__init__ in GoNasional/nodes), for per_robot.
NODE_NAMES = ('motion_manager', 'perception_bridge', 'localization', 'sim_landmark_observer',
              'game_controller', 'team_comm', 'behavior')


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('GoNasional'), 'config')
    sim = LaunchConfiguration('sim')
    common = os.path.join(config_dir, 'soccer.yaml')
    override = PythonExpression([
        "'", config_dir, "/' + ('sim.yaml' if '", sim, "' == 'true' else 'robot.yaml')"])

    def per_robot(context):
        """Launch-argument overrides as a parameters file loaded last (empty
        argument = keep the YAML value). Node-specific YAML entries beat `/**`
        ones whatever the file order, and params files beat `-p`, so the file
        names every node explicitly. Several simulated robots run this launch
        once each with their own values."""
        values = {}
        player = LaunchConfiguration('player').perform(context)
        if player:
            values['player_number'] = int(player)
        start = LaunchConfiguration('start_pose').perform(context)
        if start:
            values['start_pose'] = [float(v) for v in start.split(',')]
        goalie = LaunchConfiguration('goalie').perform(context)
        if goalie:
            values['team.goalie_player'] = int(goalie)
        if not values:
            return []
        out = tempfile.NamedTemporaryFile('w', suffix='_aroc_robot.yaml', delete=False)
        # Deep copies: rcl rejects the YAML aliases a shared dict would produce.
        yaml.safe_dump({name: {'ros__parameters': copy.deepcopy(values)}
                        for name in NODE_NAMES}, out)
        out.close()
        return [out.name]

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
        def make(context):
            return [Node(package='GoNasional', executable=executable, output='screen',
                         parameters=params + per_robot(context), **kwargs)]
        return OpaqueFunction(function=make)

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
        DeclareLaunchArgument('player', default_value='',
                              description='player_number override (empty: soccer.yaml)'),
        DeclareLaunchArgument('start_pose', default_value='',
                              description="'x,y,theta' start pose override (empty: YAML)"),
        DeclareLaunchArgument('goalie', default_value='',
                              description='behavior team.goalie_player override (empty: YAML)'),
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

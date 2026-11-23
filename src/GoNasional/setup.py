import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'GoNasional'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aroc',
    maintainer_email='98314081+nugrosss@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Legacy single-robot stack
            'state = GoNasional.main_state:main',
            'vision = GoNasional.main_vision:main',
            'jarak = GoNasional.jarak:main',
            'tunning = GoNasional.tunning_PID:main',
            'gawang = GoNasional.errorgawang:main',
            'rekam = GoNasional.record:main',
            'trigonocak = GoNasional.trigonorun:main',
            # Layered soccer stack (launch/aroc_soccer.launch.py)
            'motion_manager = GoNasional.nodes.motion_manager:main',
            'perception_bridge = GoNasional.nodes.perception_bridge:main',
            'localization = GoNasional.nodes.localization_node:main',
            'behavior = GoNasional.nodes.behavior_node:main',
            'sim_landmarks = GoNasional.nodes.sim_landmark_observer:main',
            'game_controller = GoNasional.nodes.game_controller_node:main',
            'fake_referee = GoNasional.nodes.fake_referee:main',
            'team_comm = GoNasional.nodes.team_comm:main',
            'preflight = GoNasional.nodes.preflight:main',
        ],
    },
)

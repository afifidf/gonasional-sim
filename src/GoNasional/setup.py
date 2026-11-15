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
            'bangun = GoNasional.main_action:main',
            'jalan = GoNasional.crab_walk:main',
            'button = GoNasional.sec_buttonhandler:main',
            'goNas = GoNasional.main_task:main',
            'state = GoNasional.main_state:main',
            'headcontrol = GoNasional.main_head_control:main',
            'vision = GoNasional.main_vision:main',
            'jarak = GoNasional.jarak:main',
            'cek = GoNasional.cek:main',
            'tunning = GoNasional.tunning_PID:main',
            'imu = GoNasional.hadap:main',
            'imucsv = GoNasional.imucsv:main',
            'imucek = GoNasional.imucek:main',
            'gawang = GoNasional.errorgawang:main',
            'visionSec = GoNasional.sec_cam_lokal:main',
            'deteksi = GoNasional.ilmiideteksi:main',
            'rekam = GoNasional.record:main',
            # 'kick = main_task26.main_kick:mainSecVision = GoNasional.landmark:main',
            'kirimData = GoNasional.kirimdatajarak:main',
            'trigonocak = GoNasional.trigonorun:main',
        ],
    },
)

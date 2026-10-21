import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autonav_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jatin',
    maintainer_email='jatinpatil2003@gmail.com',
    description=(
        'Always-on utility nodes: the cmd_vel safety-timeout watchdog, the '
        'system liveness heartbeat, and the SLAM-time TF-to-pose bridge.'
    ),
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_republisher = autonav_utils.cmd_vel_republisher:main',
            'tf_pose_bridge = autonav_utils.tf_pose_bridge:main',
            'system_heartbeat = autonav_utils.system_heartbeat:main',
        ],
    },
)

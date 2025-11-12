"""Setup.py for ROS2."""

from setuptools import find_packages, setup

package_name = 'motion_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch/',
            ['launch/pickplace.launch.xml'],
        ),
        (
            'share/' + package_name + '/launch/',
            ['launch/moplan.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='conorbot',
    maintainer_email='cwoodhayes@gmail.com',
    description='Motion planning wrapper on MoveIt and a demo task.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'motion_planner_integration_test = '
                'motion_planner.motion_planner:main'
            ),
            'planning_scene_test = motion_planner.planning_scene_test:main',
            'robot_state_test = motion_planner.robot_state_test:main',
            'pick_node = motion_planner.pick_node:main',
        ],
    },
)

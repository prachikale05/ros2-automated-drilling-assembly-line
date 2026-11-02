from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'assembly_line'

setup(
    name=package_name,
    version='1.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),

        (
            os.path.join('share', package_name, 'models', 'conveyor'),
            glob('models/conveyor/*')
        ),

        (
            os.path.join('share', package_name, 'models', 'workpiece'),
            [ 'models/workpiece/model.config',
                'models/workpiece/model.sdf',
                'models/workpiece/workpiece_with_two_holes.stl',
                'models/workpiece/workpiece_with_two_holes.scad']
        ),
        

        (
            os.path.join('share', package_name, 'models', 'trolley'),
            glob('models/trolley/*')
        ),
        
        (
            os.path.join('share', package_name, 'models', 'workstation1'),
            glob('models/workstation1/*')
        ),

        (
            os.path.join('share', package_name, 'description'),
            glob('description/*.xacro')
        ),

        (
            os.path.join('share', package_name, 'description', 'robot'),
            glob('description/robot/*.xacro')
        ),

        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),

        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='prachi',
    maintainer_email='prachikale005@gmail.com',

    description='ROS 2 and Gazebo Classic automated assembly line with industrial pick-and-place and two-hole drilling operations',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        'workpiece_controller = assembly_line.workpiece_controller:main',
        'workpiece_sensor = assembly_line.workpiece_sensor:main',
        'automatic_pick_place = assembly_line.automatic_pick_place:main',
        'drilling_controller = assembly_line.drilling_controller:main',
        'position_accuracy_monitor = assembly_line.position_accuracy_monitor:main',
        'position_accuracy_monitor_v2 = assembly_line.position_accuracy_monitor_v2:main',
        ],
    },
)

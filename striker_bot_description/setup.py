from setuptools import setup
import os
from glob import glob

package_name = 'striker_bot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install Config/Param Files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Install URDF/Xacro Files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        
        # Install RViz Config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Install World Files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Striker 7 Robot with GPS Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # GPS Waypoint Navigation
            'gps_waypoint_logger = striker_bot_description.gps_waypoint_logger:main',
            'logged_waypoint_follower = striker_bot_description.logged_waypoint_follower:main',
            
            # Utility scripts
            'waypoint_logger = striker_bot_description.waypoint_logger:main',
            'check_gps = striker_bot_description.check_gps:main',
            'fake_gps_publisher = striker_bot_description.fake_gps_publisher:main',
        ],
    },
)
from setuptools import find_packages, setup
import glob
import os

package_name = 'trajectory_tracking'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),

        # Corrected glob usage
        # ('share/' + package_name + '/urdf', glob.glob('urdf/*.xacro')),  # Add this line for the URDF
        # ('share/'+ package_name + '/maps', glob.glob('maps/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='host',
    maintainer_email='b22me002@iitj.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'trajectory_node = trajectory_tracking.trajectory_node:main',
            'obstacle_avoidance_node = trajectory_tracking.obstacle_avoidance_node:main',

        ],
    },
)

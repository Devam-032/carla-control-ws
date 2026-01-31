from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carla_control'

setup(
    name=package_name,
    version='0.0.0',

    # Keep your existing package discovery
    packages=find_packages(exclude=['test']),

    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pro',
    maintainer_email='pro@todo.todo',
    description='CARLA control and trajectory services',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            # Keep your existing executable
            'Ego_controller = carla_control.carla_demo1:main',
            'stanley_controller = carla_control.stanley_controller:main',
            'trajectory_service = carla_control.trajectory_service:main',
            'lqr_controller = carla_control.lqr_controller:main',
        ],
    },
)

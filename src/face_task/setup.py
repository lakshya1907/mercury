from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'face_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='WP-2 face detection pipeline for Mercury UGVC',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'face_recognition       = face_task.face_recognition_node:main',
            'face_task              = face_task.face_task_node:main',
            'turret_controller      = face_task.turret_controller_node:main',
            'face_task_trigger      = face_task.face_task_trigger_node:main',
            # Sim-only: drives robot to WP-2 then triggers face detection
            'sim_waypoint_nav       = face_task.sim_waypoint_nav_node:main',
        ],
    },
)

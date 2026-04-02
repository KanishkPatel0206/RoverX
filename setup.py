import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register the package in the ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        
        # --- INCLUDE YOUR FILES ---
        # 1. Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 2. URDF Files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # 3. Config Files (RViz, Bridge)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # --------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='3-Wheeled Robot Description Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_wasd = my_robot_description.teleop_node:main',
            'yolo        = my_robot_description.yolo:main',      # ← added
            'cam         = my_robot_description.cam:main',       # ← added
        ],
    },
)
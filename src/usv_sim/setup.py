from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'usv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Required index resource
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        
        # URDF/Xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # RViz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mobseap',
    maintainer_email='BASSOT@my.erau.edu',
    description='Simple RViz-based USV Simulation using /cmd_vel',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_usv_sim_node = usv_sim.simple_usv_sim_node:main'
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lucia_tof'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LUCIA Team',
    maintainer_email='noeltem98@gmail.com',
    description='Publishes the Pico 2 VL53L5CX ToF sensor frames for LUCIA.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_node = lucia_tof.tof_node:main',
        ],
    },
)

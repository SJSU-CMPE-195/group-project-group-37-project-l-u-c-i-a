from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lucia_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LUCIA Team',
    maintainer_email='christianmichael.villanueva@sjsu.edu',
    description='Launch files for bringing up the full LUCIA rover stack.',
    license='TODO',
    entry_points={
        'console_scripts': [],
    },
)

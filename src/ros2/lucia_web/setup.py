from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lucia_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LUCIA Team',
    maintainer_email='christianmichael.villanueva@sjsu.edu',
    description='Web control page: mapping, live map and radar, explore / return home, speed, drive pad, save maps.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'web_ui = lucia_web.web_ui:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'lucia_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LUCIA Team',
    maintainer_email='christianmichael.villanueva@sjsu.edu',
    description='Roomba bridge node — translates ROS2 /cmd_vel to Roomba OI serial commands.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'roomba_bridge = lucia_control.roomba_bridge:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'lucia_imu'

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
    description='IMU bridge node: reads the BNO08X Pico over USB serial and publishes /imu/data.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'imu_bridge = lucia_imu.imu_bridge:main',
        ],
    },
)

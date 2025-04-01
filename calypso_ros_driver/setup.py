from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'calypso_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='247340',
    maintainer_email='mblaha29@seznam.cz',
    description='UART calypso ultralowpower ultrasonic windmeter driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calypso_driver = calypso_ros_driver.calypso_driver:main'
        ],
    },
)



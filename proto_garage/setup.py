from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'proto_garage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'database'), glob('database/*')),
        
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnaud',
    maintainer_email='arnaud.sibenaler@cpe.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_detection = proto_garage.car_detection:main',
            'read_plate = proto_garage.read_plate:main',
            'tof_node = proto_garage.ToFNode:main',
            'door_controller = proto_garage.door_controller:main',
            'plate_checker = proto_garage.plate_checker:main',
            'servonode = proto_garage.ServoNode:main',
        ],
    },
)

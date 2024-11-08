from setuptools import find_packages, setup

package_name = 'proto_garage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
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
            'tof_node = proto_garage.ToFNode:main',
        ],
    },
)

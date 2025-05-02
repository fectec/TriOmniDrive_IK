from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'triomnidrive_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['triomnidrive_control', 'triomnidrive_control.*']),
    package_data={
        'triomnidrive_control': ['proto/*.py'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fectec',
    maintainer_email='fectec151@gmail.com',
    description='ROS2 package for controlling the three-wheeled omnidirectional robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grpc_client = triomnidrive_control.grpc_client:main',        
        ],
    },
)
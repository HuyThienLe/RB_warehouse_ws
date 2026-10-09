from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wh_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lehuythien',
    maintainer_email='lehuythien1702@gmail.com',
    description='Thị giác và Visual Servoing cho hệ thống AGV kho hàng',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector_node = wh_vision.tag_detector_node:main',
            'dock_server = wh_vision.dock_server:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'wh_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/schema.sql',
            'config/items_seed.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/task_manager.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lehuythien',
    maintainer_email='lehuythien1702@gmail.com',
    description='Task manager and warehouse inventory database',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager_node = wh_task_manager.task_manager_node:main',
        ],
    },
)

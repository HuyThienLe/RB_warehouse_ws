from setuptools import find_packages, setup

package_name = 'wh_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web.launch.py']),
    ],
    package_data={
        'wh_web': ['static/*', 'static/**/*'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lehuythien',
    maintainer_email='lehuythien1702@gmail.com',
    description='Warehouse local web management portal and REST/WebSocket API',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
            'httpx',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_server = wh_web.app:main',
        ],
    },
)

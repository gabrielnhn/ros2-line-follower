from setuptools import setup

package_name = 'ta19_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Allan Cedric',
    maintainer_email='allan.py3000@gmail.com',
    description='Scripts used to teleoperate on a pair of motors',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator = ta19_ros2.motor_actuator:main',
            'teleop = ta19_ros2.motor_teleop:main',
        ],
    },
)

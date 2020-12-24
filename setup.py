from setuptools import setup

package_name = 'follower'

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
    maintainer='Gabriel Nascarella Hishida',
    maintainer_email='gabrielnhn@ufpr.br',
    description='Have a turtlebot follow a Robotrace track ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = follower.follower_node:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'bp_control'

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
    maintainer='Johan Ramongali',
    maintainer_email='johankgotso7@gmail.com',
    description='control your robot using arduino serial without micro ros installed on arduino',
    license='open source',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'robot_control_node = bp_control.robot_control_node:main',
           'wheel_tf_node = bp_control.wheel_tf_node:main',  
           'odom_node = bp_control.odom_node:main',
        ],
    },
)

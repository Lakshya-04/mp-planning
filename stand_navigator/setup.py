from setuptools import setup

package_name = 'stand_navigator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'stand_navigator_node = stand_navigator.ros_node:main',
        'wall_docking_node = stand_navigator.wall_docking_node:main']},
    maintainer='Lakshya Agarwal',
    maintainer_email='100lakshyaagarwal@gmail.com',
    description='Drive the base to a reachability stand',
    license='Apache-2.0',
    tests_require=['pytest'],
)

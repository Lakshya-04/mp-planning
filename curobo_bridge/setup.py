from setuptools import setup

package_name = 'curobo_bridge'

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
    entry_points={'console_scripts': ['curobo_planner_node = curobo_bridge.ros_node:main']},
    maintainer='Lakshya Agarwal',
    maintainer_email='100lakshyaagarwal@gmail.com',
    description='cuRobo v2 behind a ROS-free planner core',
    license='Apache-2.0',
    tests_require=['pytest'],
)

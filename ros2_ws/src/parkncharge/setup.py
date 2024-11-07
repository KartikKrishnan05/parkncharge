from setuptools import find_packages, setup

package_name = 'parkncharge'

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
    maintainer='krishnan',
    maintainer_email='krishnan@iabg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller_node = own_teleop_pkg.controller:main",
            "gazebo_ros_launch = parkncharge.gazebo_ros_launch:main",
            "odom_subscriber_node = parkncharge.read_odomentry:main",
            "move_and_monitor_vehicle = parkncharge.MoveAndMonitor:main",
            "autonomous_drive = parkncharge.autonomousDrive:main",
            "main = parkncharge.main:main",
        ],
    },
)

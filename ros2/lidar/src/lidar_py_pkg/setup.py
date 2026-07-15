from glob import glob

from setuptools import find_packages, setup

package_name = "lidar_py_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@example.com",
    description="Python ROS2 LaserScan publisher for the serial lidar.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_py_node = lidar_py_pkg.lidar_py_node:main",
            "uart_raw_dump = lidar_py_pkg.uart_raw_dump:main",
            "wheel_odom_node = lidar_py_pkg.wheel_odom_node:main",
            "base_driver_node = lidar_py_pkg.wheel_odom_node:main",
            "camera_node = lidar_py_pkg.camera_node:main",
        ],
    },
)

from glob import glob

from setuptools import find_packages, setup

package_name = "rectangle_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="heximiao",
    maintainer_email="heximiao@example.com",
    description="Black rectangle detector and Foxglove camera publisher.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rectangle_vision_node = rectangle_vision.rectangle_vision_node:main",
            "gimbal_uart_node = rectangle_vision.gimbal_uart_node:main",
        ],
    },
)

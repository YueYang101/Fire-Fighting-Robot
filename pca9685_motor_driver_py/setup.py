from setuptools import setup
from glob import glob
import os

package_name = "pca9685_motor_driver_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Yang Yue",
    author_email="zcemuex@ucl.ac.uk",
    description="ROS 2 motor driver for a PCA9685 PWM board",
    license="MIT",
    entry_points={
        "console_scripts": [
            # ros2 run pca9685_motor_driver_py motor_driver_node
            f"motor_driver_node = {package_name}.motor_driver_node:main",
        ],
    },
    # non-python resources that must be installed
    data_files=[
        ("share/ament_index/resource_index/packages",
            [f"resource/{package_name}"]),
        (f"share/{package_name}",              ["package.xml"]),
        (f"share/{package_name}/launch",       glob("launch/*.py")),
        (f"share/{package_name}/config",       glob("config/*.yaml")),
        (f"share/{package_name}/srv",          glob("srv/*.srv")),
    ],
)

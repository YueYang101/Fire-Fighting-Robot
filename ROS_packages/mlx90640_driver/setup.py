from setuptools import setup
import os
from glob import glob

package_name = "mlx90640_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    # ------------------------------------------------------------------
    # Files that should be copied into the install tree
    # ------------------------------------------------------------------
    data_files=[
        # ① package-level resource (already present)
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),

        # ② executable-level resource  (<<< NEW line)
        ("share/ament_index/resource_index/ros2_executable",
         ["resource/thermal_camera_node"]),

        # ③ misc package data
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    # ------------------------------------------------------------------
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="ROS 2 driver for MLX90640 thermal camera",
    license="Apache-2.0",
    tests_require=["pytest"],
    # ------------------------------------------------------------------
    # Entry-point script ROS 2 should expose via `ros2 run`
    # ------------------------------------------------------------------
    entry_points={
        "console_scripts": [
            "thermal_camera_node = mlx90640_driver.thermal_camera_node:main",
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ad_juno"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="canozaydin",
    maintainer_email="ozaydincan.app@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "can_node_resurrections = ad_juno.can_node_resurrections:main",  # Entry point for can_node_resurrections.py
            "stop_node = ad_juno.stop_node:main",
            "path_planning = ad_juno.path_planning:main",
            "seg_node = ad_juno.seg_node:main",
            "steering_brake_node = ad_juno.steering_brake_node:main",
            "throttle_node = ad_juno.throttle_node:main",
            "zed_node = ad_juno.zed_node:main",
            "zed_publisher = ad_juno.zed_publisher:main",
        ],
    },
)

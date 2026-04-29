import os
from glob import glob

from setuptools import find_packages, setup

package_name = "umi_dex_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    scripts=["scripts/record.sh"],
    zip_safe=True,
    maintainer="Linkerbot Maintainers",
    maintainer_email="helloworld@linkerbot.cn",
    description="ROS2 Jazzy capture pipeline for UMI-Dex.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "can_raw_node = umi_dex_bringup.can_raw_node:main",
            "usart_raw_node = umi_dex_bringup.usart_raw_node:main",
            "interactive_capture_node = umi_dex_bringup.interactive_capture_node:main",
        ],
    },
)

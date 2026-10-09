from setuptools import find_packages, setup
from glob import glob
import os

package_name = "sideboard_p2_fusion"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial", "PyYAML"],
    zip_safe=True,
    maintainer="sirobo",
    maintainer_email="dev@local.invalid",
    description="Slip-aware wheel/LIO/GNSS fusion for the hoverboard sideboard P2 stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "p2_fusion_node = sideboard_p2_fusion.p2_fusion_node:main",
            "serial_aiding_bridge = sideboard_p2_fusion.serial_aiding_bridge:main",
        ],
    },
)

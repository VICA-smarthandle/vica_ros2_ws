import os
from glob import glob

from setuptools import setup

package_name = "vica_mission_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tony",
    maintainer_email="devlee792458@gmail.com",
    description="VICA mission manager: /vica/intent 게이트 심사 후 Nav2 goal 발행",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager = vica_mission_manager.mission_manager_node:main",
            "emergency_estop_bridge = vica_mission_manager.emergency_estop_bridge:main",
        ],
    },
)

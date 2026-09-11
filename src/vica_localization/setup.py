import os
from glob import glob

from setuptools import find_packages, setup


package_name = "vica_localization"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VICA Team",
    maintainer_email="2561110043@office.kopo.ac.kr",
    description="VICA wheel odometry and EKF localization bringup",
    license="Apache-2.0",
    tests_require=["pytest"],
)

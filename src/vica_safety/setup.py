import os
from glob import glob

from setuptools import find_packages, setup


package_name = "vica_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ji_w",
    maintainer_email="2561110043@office.kopo.ac.kr",
    description="Independent VICA software safety layer.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "emergency_stop_node = vica_safety.emergency_stop_node:main",
            "safety_supervisor_node = vica_safety.safety_supervisor_node:main",
            "app_emergency_node = vica_safety.app_emergency_node:main",
        ],
    },
)

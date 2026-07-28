import os
from glob import glob

from setuptools import find_packages, setup


package_name = "vica_user_guidance"

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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ji_w",
    maintainer_email="2561110043@office.kopo.ac.kr",
    description="Smart Handle user guidance layer for VICA.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "turn_guide_node = vica_user_guidance.turn_guide_node:main",
            "user_guidance_driver_node = "
            "vica_user_guidance.user_guidance_driver_node:main",
        ],
    },
)

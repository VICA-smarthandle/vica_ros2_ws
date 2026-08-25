from setuptools import setup

package_name = "vica_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minsmart",
    maintainer_email="mjw41177@gmail.com",
    description="VICA 사람 인지: 탐지 시계열에서 stable·approachable 판정",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_detector_node = vica_perception.person_detector_node:main",
        ],
    },
)

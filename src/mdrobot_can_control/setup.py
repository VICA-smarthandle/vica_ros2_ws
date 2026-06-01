from setuptools import find_packages, setup

package_name = 'mdrobot_can_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ji_w',
    maintainer_email='2561110043@office.kopo.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['keyboard_knob = mdrobot_can_control.mdrobot_can_keyboard_knob_node:main',
        ],
    },
)

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'vica_system_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minsmart',
    maintainer_email='mjw41177@gmail.com',
    description='VICA 전체 상태 감시와 앱 진단 표시용 관측 계층.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_health_monitor_node ='
            ' vica_system_monitor.robot_health_monitor_node:main',
            'external_diagnostics_node ='
            ' vica_system_monitor.external_diagnostics_node:main',
        ],
    },
)

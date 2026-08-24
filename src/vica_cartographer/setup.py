from glob import glob
import os

from setuptools import setup

package_name = 'vica_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ji_w',
    maintainer_email='2561110043@office.kopo.ac.kr',
    description='VICA Cartographer 2D mapping configuration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_preview_node ='
            ' vica_cartographer.map_preview_node:main',
            'mapping_supervisor_node ='
            ' vica_cartographer.mapping_supervisor_node:main',
        ],
    },
)

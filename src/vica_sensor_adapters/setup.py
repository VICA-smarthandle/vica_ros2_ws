from setuptools import find_packages, setup

package_name = 'vica_sensor_adapters'

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
    maintainer_email='rlaalstj2954@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "vslam_covariance_adapter = vica_sensor_adapters.vslam_covariance_adapter:main",
            "imu_base_link_adapter = vica_sensor_adapters.imu_base_link_adapter:main",
            "scan_rear_filter = vica_sensor_adapters.scan_rear_filter:main",
        ],
    },
)

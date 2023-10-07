from setuptools import find_packages, setup
from glob import glob

package_name = 'zed_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/**/*', recursive=True)),
        ('share/' + package_name + '/launch', glob('launch/**/*', recursive=True))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Balachandra Bhat',
    maintainer_email='bnbhat311@gmail.com',
    description='Minimal ROS2 Driver for ZED Camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_ros2_driver = zed_driver.zed_ros_driver:main',
        ],
    },
)

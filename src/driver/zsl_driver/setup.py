from setuptools import find_packages, setup
import os

package_name = 'zsl_driver'


def _recursive_data_files(data_dir):
    paths = []
    for root, dirs, files in os.walk(data_dir):
        for f in files:
            paths.append(os.path.join(root, f))
    install_dir = os.path.join('share', package_name, data_dir)
    return (install_dir, paths)


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'zsl_driver': ['sdk_lib/*.so']},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        _recursive_data_files('launch'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@local',
    description='ZSL-1W wheel-foot robot ROS 2 driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'zsl_driver_node = zsl_driver.zsl_driver_node:main',
        ],
    },
)

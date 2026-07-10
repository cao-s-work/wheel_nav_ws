from setuptools import find_packages, setup

package_name = 'serial_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加这一行，将 launch 文件夹复制到 share/serial_node 下
        ('share/' + package_name + '/launch', ['launch/serial_comm.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsh',
    maintainer_email='hsh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_twist_publisher = serial_node.serial_twist_publisher:main',
        ],
    },
)

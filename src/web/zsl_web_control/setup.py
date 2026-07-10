from setuptools import setup

package_name = 'zsl_web_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zsl_web_control.launch.py']),
        ('share/' + package_name + '/config', ['config/web_control.yaml']),
        ('share/' + package_name + '/static', [
            'static/index.html',
            'static/app.js',
            'static/style.css',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@localhost.local',
    description='ZSL-1W Web control gateway',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_control_node = zsl_web_control.web_node:main',
        ],
    },
)

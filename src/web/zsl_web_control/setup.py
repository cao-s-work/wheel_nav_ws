from glob import glob
from setuptools import find_packages, setup

package_name = "zsl_web_control"

setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/static", glob("static/*")),
        ("share/" + package_name + "/hook", glob("hook/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@localhost",
    description="Commercial ZSL-1W web operations console",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "web_control_node = zsl_web_control.web_node:main",
            "topic_rate_monitor = zsl_web_control.topic_rate_monitor:main",
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="andbondstyle@gmail.com",
    description="detection",
    license="MIT",
    extras_require={},
    entry_points={
        "console_scripts": [
            "detection = detection.detection:main"
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "voice"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="email@example.com",
    description="voice",
    license="MIT",
    extras_require={},
    entry_points={
        "console_scripts": [
            "stt_node = voice.stt_node:main",
            "tts_node = voice.tts_node:main",
        ],
    },
)

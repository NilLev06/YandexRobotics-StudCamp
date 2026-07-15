from setuptools import find_packages, setup

package_name = "hwnode"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="andbondstyle@gmail.com",
    description="hwnode",
    license="MIT",
    extras_require={},
    entry_points={
        "console_scripts": [
            "hwnode = hwnode.hwnode:main"
        ],
    },
)

from setuptools import find_packages, setup

package_name = "aruco_node"

setup(
    name=package_name,
    version="0.0.0",
    python_requires=">=3.13",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    maintainer="Tyler Roman",
    maintainer_email="tyler73750@gmail.com",
    description="TODO",
    license="TODO",
    entry_points={
        "console_scripts": [
            "aruco_node = aruco_node.main:main",
            "image_capture = aruco_node.image_capture:main",
        ],
    },
)

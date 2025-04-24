from setuptools import find_packages, setup

package_name = "simulator"

_ = setup(
    name=package_name,
    version="0.0.0",
    python_requires=">=3.13",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/gazebo_only.launch.py"]),
        ("share/" + package_name + "/resource", ["resource/world.sdf.xml"]),
        (
            "share/" + package_name + "/resource",
            ["resource/rover.urdf.xacro.xml"],
        ),
        ("share/" + package_name + "/params", ["params/bridge.yaml"]),
        ("share/" + package_name + "/params", ["params/nav2.yaml"]),
    ],
    zip_safe=True,
    maintainer="Barrett Ray",
    maintainer_email="contact@barretts.club",
    description="TODO",
    license="TODO",
    entry_points={
        "console_scripts": [
            "simulator = simulator.main:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "soro_simulator_driver"

setup(
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
    ],
    zip_safe=True,
    maintainer="brendan",
    maintainer_email="bford@axmilius.com",
    description="TODO",
    license="TODO",
    entry_points={
        "console_scripts": [
            "log_node = log_node.main:main",
        ],
    },
)

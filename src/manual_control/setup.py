from setuptools import find_packages, setup

package_name: str = "manual_control"
manual_control_gui_node_name: str = "manual_control_gui_node"

_ = setup(
    name=package_name,
    version="1.0.0",
    #
    # boilerplate; don't change this
    install_requires=["setuptools"],
    tests_require=["pytest"],
    packages=find_packages(exclude=["test"]),
    #
    # stuff to add into `install/`
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    #
    # this lists things we can use w/ `ros2 run manual_control <thing>`
    entry_points={
        "console_scripts": [
            f"{manual_control_gui_node_name} = {manual_control_gui_node_name}.main:main",
        ],
    },
)

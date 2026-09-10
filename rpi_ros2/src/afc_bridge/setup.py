from setuptools import setup

package_name = "afc_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Matt McCrink",
    maintainer_email="mccrink@example.edu",
    description="Pi ROS2 supervisor/bridge between MAVROS and the RP2350 valve node.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = afc_bridge.bridge_node:main",
        ],
    },
)

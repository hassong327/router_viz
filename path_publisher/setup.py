from setuptools import setup

package_name = "path_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/path_publisher.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="songha",
    maintainer_email="eiffeltower1206+fmcl1@kookmin.ac.kr",
    description="Standalone exact-Bezier PH path publisher.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_publisher_node = path_publisher.path_publisher_node:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "qcar2_control_2"

setup(
    name=package_name,
    version="0.9.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],

    zip_safe=True,
    maintainer="jeon_semin",
    maintainer_email="eiffeltower1206@kookmin.ac.kr",
    description="ACC control nodes",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ph_path_publisher = qcar2_control_2.ph_path_publisher:main",
            "dashboard_viewer = qcar2_control_2.dashboard_viewer:main",
        ],
    },
)

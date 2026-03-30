from setuptools import find_packages, setup


package_name = "fallitis_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/stack.launch.py"]),
        (f"share/{package_name}/config", ["config/stack.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sim",
    maintainer_email="sim@localhost",
    description="Launch files for the Fall-Itis ROS 2 support stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sync_controller = fallitis_bringup.sync_controller:main",
        ],
    },
)

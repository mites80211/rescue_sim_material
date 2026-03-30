from setuptools import find_packages, setup


package_name = "fallitis_common"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sim",
    maintainer_email="sim@localhost",
    description="Shared helpers for Fall-Itis ROS 2 packages.",
    license="MIT",
)

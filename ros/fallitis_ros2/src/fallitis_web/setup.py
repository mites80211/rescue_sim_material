from setuptools import find_packages, setup


package_name = "fallitis_web"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/static",
            [
                "fallitis_web/static/index.html",
                "fallitis_web/static/app.js",
                "fallitis_web/static/styles.css",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sim",
    maintainer_email="sim@localhost",
    description="Web GUI and browser teleop for the Fall-Itis rescue robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_server = fallitis_web.web_server:main",
        ],
    },
)

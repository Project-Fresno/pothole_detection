from setuptools import setup

package_name = "pothole_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name + "/launch", ["launch/cov_to_pcl_launch.py"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yas",
    maintainer_email="yas@todo.todo",
    description="Pothole detection node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pothole_detection = pothole_detection.pothole_detection:main",
        ],
    },
)

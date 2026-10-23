from setuptools import setup
from glob import glob
import os

package_name = "omni_drive"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="local project",
    maintainer_email="user@example.com",
    description="Standalone holonomic joystick drive using unmodified Blitz serial framing.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["joy_to_velocity = omni_drive.joy_to_velocity:main"]},
)

"""ament_python setup for the pragmabot package (ported from catkin).

The upstream package was catkin (ROS 1): a CMakeLists.txt plus a setup.py
calling catkin_pkg.generate_distutils_setup. Both are gone - colcon builds
this as a plain ament_python package now.

`package_dir={"": "src"}` is kept from upstream, so the import path stays
`from pragmabot.vlm_task_planner import ...` and none of the 14 untouched
VLM/memory modules need editing.
"""

import os
from glob import glob

from setuptools import setup

package_name = "pragmabot"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    package_dir={"": "src"},
    # nodes/ lives OUTSIDE src/pragmabot/, so it is not importable as
    # pragmabot.nodes and cannot be a console_scripts entry point. Install
    # the scripts directly instead - this also keeps the upstream layout,
    # so the 14 untouched modules stay exactly where they were.
    scripts=["nodes/pragmabot_node.py"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        # config.yaml must be installed, not just present in the source tree:
        # get_package_path() resolves against the install space at runtime.
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harish Deepak",
    maintainer_email="harish.deepak.c@gmail.com",
    description="PragmaBot: Learning to plan tasks by experiencing the real world (FR3 port)",
    license="BSD-3-Clause",
    tests_require=["pytest"],
)

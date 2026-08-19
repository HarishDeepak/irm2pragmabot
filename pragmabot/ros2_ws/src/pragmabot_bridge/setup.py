from setuptools import find_packages, setup

package_name = "pragmabot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Harish Deepak",
    maintainer_email="harish.deepak@example.com",
    description=(
        "Action-client bridge: routes PragmaBot planner skill decisions to "
        "Container 1's live MoveIt2/gripper interfaces (/move_action, "
        "/compute_cartesian_path, /franka_gripper/grasp)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = pragmabot_bridge.bridge_node:main",
        ],
    },
)

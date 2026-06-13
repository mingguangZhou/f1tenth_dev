from setuptools import setup

package_name = "rl_speed_inference"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/rl_speed_inference.yaml",
            "config/rl_speed_inference_sim.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/rl_speed_inference_launch.py",
            "launch/rl_speed_inference_sim_launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="PPO speed inference node for F1TENTH",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ppo_speed_node = rl_speed_inference.ppo_speed_node:main",
        ],
    },
)

from setuptools import find_packages, setup

package_name = "policy_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Anonymous Authors",
    maintainer_email="anonymous@users.noreply.github.com",
    description="GUI-based natural-language policy interface for Nav2.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "policy_bridge_gui = policy_bridge.policybridge_gui:main",
        ],
    },
)

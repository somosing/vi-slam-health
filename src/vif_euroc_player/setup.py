from setuptools import find_packages, setup

package_name = "vif_euroc_player"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Mohit Singh",
    maintainer_email="mohit.singh@users.noreply.github.com",
    description="Timestamp-faithful EuRoC stereo-inertial publisher for VIF.",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [
            "euroc_player = vif_euroc_player.euroc_player:main",
        ],
    },
)

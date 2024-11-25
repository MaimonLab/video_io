from setuptools import setup

import glob

package_name = "video_io"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="James Ryu",
    maintainer_email="jyu01@rockefeller.edu",
    description=(
        "General use nodes that play or save video files."
    ),
    license="LGPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "video_saver = video_io.video_saver:main",
            "video_player = video_io.video_player:main",
            "burst_video_saver = video_io.burst_video_saver:main",
            "simple_burst_publisher = video_io.simple_burst_publisher:main",
        ],
    },
)

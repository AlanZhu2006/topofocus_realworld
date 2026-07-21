from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "semantic_mapping"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TinyNav Maintainers",
    maintainer_email="alan@example.com",
    description="Timestamped TinyNav-pose-conditioned RGB-D mapping tools.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "semantic_perception_node = semantic_mapping.semantic_perception_node:main",
            "semantic_pointcloud_node = semantic_mapping.semantic_pointcloud_node:main",
            "occupancy_mapper_node = semantic_mapping.occupancy_mapper_node:main",
            "semantic_mapper_node = semantic_mapping.semantic_mapper_node:main",
        ],
    },
)

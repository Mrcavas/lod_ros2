from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mag_corrector"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mrcavas",
    maintainer_email="sidorov.mrcavas@mail.ru",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "corrector = mag_corrector.node:main",
        ],
    },
)

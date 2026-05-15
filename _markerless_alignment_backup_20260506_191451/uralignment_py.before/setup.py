from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'uralignment_py'

def package_files(directory):
    paths = [
        "scripts/realsense_data_publisher.py",
        "scripts/recorded_data_publisher.py",
        "scripts/markerless_pose_estimator.py",
        "scripts/markerless_pose_averager.py",
        "user_interface/plotter.py",
        "user_interface/display.py",
    ]
    if not os.path.isdir(directory):
        return paths
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f"share/{package_name}/launch", glob("launch/*")),
        (f"share/{package_name}/config", package_files("config") if os.path.isdir("config") else []),
        (f"share/{package_name}/data", package_files("data") if os.path.isdir("data") else []),
        (f"share/{package_name}/models", package_files("models") if os.path.isdir("models") else []),
        (f"share/{package_name}/video_flange", package_files("video_flange") if os.path.isdir("video_flange") else []),
        (f"share/{package_name}/video_flange_fake", package_files("video_flange_fake") if os.path.isdir("video_flange_fake") else []),
        (f"share/{package_name}/video_cavity", package_files("video_cavity") if os.path.isdir("video_cavity") else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nickolas Giffen',
    maintainer_email='nickolas.giffen@outlook.com',
    description='Autonomous Alignment to Four AprilTags Creating a Concentric Center as Target for Eye-in-Hand d435iRealSense Mounted to UR16e at Fermilab',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "realsense_data_publisher = ur_alignment_py.realsense_data_publisher:main",
            "recorded_data_publisher = ur_alignment_py.recorded_data_publisher:main",
            "markerless_pose_estimator = ur_alignment_py.markerless_pose_estimator:main",
            "markerless_pose_averager = ur_alignment_py.markerless_pose_averager:main",
            "user_interface = ur_alignment_py.user_interface:main",
        ],
    },
)

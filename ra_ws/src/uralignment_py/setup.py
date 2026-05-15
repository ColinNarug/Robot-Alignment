from pathlib import Path
from setuptools import find_packages, setup

package_name = 'uralignment_py'


def recursive_files(directory: str):
    root = Path(directory)
    if not root.is_dir():
        return []
    return [str(p) for p in root.rglob('*') if p.is_file()]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config', recursive_files('config')),
        (f'share/{package_name}/models', recursive_files('models')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nickolas Giffen',
    maintainer_email='nickolas.giffen@outlook.com',
    description='Markerless flange pose estimator node for Robot-Alignment.',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'markerless_pose_estimator = uralignment_py.markerless_pose_estimator:main',
        ],
    },
)

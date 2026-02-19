from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mine_explorer_resources'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/trajectories', glob('trajectories/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Théo Morel',
    maintainer_email='theo.morel195@gmail.com',
    description='Resources and robot trajectory executor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_mover = mine_explorer_resources.robot_mover:main',
        ],
    },
)
import os
from glob import glob
from setuptools import setup

package_name = 'vicon_position_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asrl',
    maintainer_email='joezou5555@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pose_data_save = vicon_position_bridge.pose_data_save:main",
            "pose_pub = vicon_position_bridge.pose_pub:main",
            "pose_sub = vicon_position_bridge.pose_sub:main",
            "pose_grapher_xy = vicon_position_bridge.pose_grapher_xy:main",
            "pose_grapher_yz = vicon_position_bridge.pose_grapher_yz:main",
            "pose_grapher_xz = vicon_position_bridge.pose_grapher_xz:main",
            "pose_grapher_xyz = vicon_position_bridge.pose_grapher_xyz:main",
            "fake_pose_pub = vicon_position_bridge.fake_pose_pub:main"
        ],
    },
)

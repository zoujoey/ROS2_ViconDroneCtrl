from setuptools import setup

package_name = 'drone_offboard_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "offboard_hover = drone_offboard_missions.offboard_hover:main",
            "offboard_ex = drone_offboard_missions.offboard_ex:main",
            "offboard_control = drone_offboard_missions.offboard_control:main",
            "offboard_control_L = drone_offboard_missions.offboard_control_L:main",
            "offboard_control_square = drone_offboard_missions.offboard_control_square:main",
            "offboard_control_circle = drone_offboard_missions.offboard_control_circle:main",
            "offboard_control_circle_tilted = drone_offboard_missions.offboard_control_circle_tilted:main",
            "offboard_control_helix = drone_offboard_missions.offboard_control_helix:main",
        ],
    },
)

import os
from glob import glob
from setuptools import setup


package_name = 'drone_keyboard_controller'

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
            "offboard_control = drone_keyboard_controller.offboard_control:main",
            "command_control = drone_keyboard_controller.command_control:main",
            "keyboard_controller = drone_keyboard_controller.keyboard_controller:main",
            "set_path_hover = drone_keyboard_controller.set_path_hover:main",
            "set_path_return = drone_keyboard_controller.set_path_return:main",
            "set_path_square = drone_keyboard_controller.set_path_square:main",
            "set_path_circle = drone_keyboard_controller.set_path_circle:main",
            "set_path_helix = drone_keyboard_controller.set_path_helix:main",
            "set_path_linear_setpoint = drone_keyboard_controller.set_path_linear_setpoint:main",
            "set_path_continuous_setpoint = drone_keyboard_controller.set_path_continuous_setpoint:main"
        ],
    },
)

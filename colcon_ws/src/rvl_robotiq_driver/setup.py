from setuptools import find_packages, setup

package_name = 'rvl_robotiq_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minh Tram',
    maintainer_email='minh.tram@mavs.uta.edu',
    description='Control package for Robotiq 2F Gripper',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        # tie executable name for ros2 run [package] [executable] to the main
        # function of the node file under src/[package]/[node_file.py]
        'console_scripts': [
            'robotiq_controller = rvl_robotiq_driver.robotiq_2f_85_node:main',
        ],
    },
)

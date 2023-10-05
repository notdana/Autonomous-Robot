from setuptools import find_packages, setup

package_name = 'tebo_main_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tubo',
    maintainer_email='devbot3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mqtt_x=tebo_main_nodes.mqtt:main",
            "process_handler_x=tebo_main_nodes.process_handler:main",
            "docking_x=tebo_main_nodes.docking:main",
            "tilt_x=tebo_main_nodes.tilt:main",
            "nav2_remapper_x=tebo_main_nodes.nav2_remapper:main",
            "return_home_x=tebo_main_nodes.return_home:main",
            "battery_x=tebo_main_nodes.battery:main",
            "leds_x=tebo_main_nodes.leds:main",
            "tebo_aura_x=tebo_main_nodes.tebo_aura:main"
        ],
    },
)

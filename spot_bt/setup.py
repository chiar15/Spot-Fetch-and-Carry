from setuptools import find_packages, setup

package_name = 'spot_bt'

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
    maintainer='chiara',
    maintainer_email='chiara.ferraioli02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "detector = spot_bt.ros_nodes.detection_node:main",
            "initialization = spot_bt.ros_nodes.initialization_node:main",
            "mission = spot_bt.script:main",
            "custom_navigation = spot_bt.ros_nodes.custom_navigation_node:main",
        ],
    },
)

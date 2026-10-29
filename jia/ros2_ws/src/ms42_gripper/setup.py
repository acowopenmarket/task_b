from glob import glob
from setuptools import find_packages, setup

package_name = 'ms42_gripper'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='MS42 Gripper Maintainer',
    maintainer_email='maintainer@example.com',
    description='MS42DC gripper serial driver',
    license='MIT',
    entry_points={'console_scripts': [
        'gripper_node = ms42_gripper.node:main',
        'protocol_cli = ms42_gripper.cli:main',
    ]},
)

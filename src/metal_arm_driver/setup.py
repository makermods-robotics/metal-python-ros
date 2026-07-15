import os
from glob import glob
from setuptools import setup

package_name = 'metal_arm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.effector'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MakerMods',
    maintainer_email='dev@makermods.ai',
    description='Metal arm driver node (metal_sdk backend)',
    license='Business',
    entry_points={
        'console_scripts': [
            'metal_arm_driver = metal_arm_driver.metal_arm_driver_single_node:main',
        ],
    },
)

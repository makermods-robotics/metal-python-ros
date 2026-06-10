import os
from glob import glob
from setuptools import setup

package_name = 'metal_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        # Required so ROS 2 can discover this package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install package.xml into share/package.
        (os.path.join('share', package_name), ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaofan zhang',
    maintainer_email='601081321@qq.com',
    description='Metal arm controller package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Register the Python script as an executable node
            # name = module.path:main
            'metal_controller = metal_controller.metal_controller:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's6_py_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardo',
    maintainer_email='eduardodavila94@hotmail.com',
    description='URDF and Launch files, and RVIZ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cylinder_exe = s6_py_urdf.cylinder:main",
            "wheel_exe = s6_py_urdf.wheel:main",
            "r2d2_exe = s6_py_urdf.r2d2:main",
            "rdk_x3_robot_exe = s6_py_urdf.rdk_x3_robot:main"
        ],
    },
)

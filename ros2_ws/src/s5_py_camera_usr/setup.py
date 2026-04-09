from setuptools import find_packages, setup

package_name = 's5_py_camera_usr'

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
    maintainer='eduardo',
    maintainer_email='eduardodavila94@hotmail.com',
    description='Your package description here',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_user_exe = s5_py_camera_usr.camera_usr:main',
        ],
    },
)

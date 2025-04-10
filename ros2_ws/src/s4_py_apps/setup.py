from setuptools import find_packages, setup

package_name = 's4_py_apps'

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
    description='-Your package description here',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_publisher_exe = s4_py_apps.publisher:main",
            "py_subscriber_exe = s4_py_apps.subscriber:main",
            "py_server_exe = s4_py_apps.server:main",
            "py_client_exe = s4_py_apps.client:main"
        ],
    },
)

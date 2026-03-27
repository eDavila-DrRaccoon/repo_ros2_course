from setuptools import find_packages, setup

package_name = 's2_py_pubsub'

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
            "publisher_exe = s2_py_pubsub.publisher:main",
            "subscriber_exe = s2_py_pubsub.subscriber:main"
        ],
    },
)

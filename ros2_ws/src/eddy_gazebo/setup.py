import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'eddy_gazebo'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('worlds/*')),
        (os.path.join('share', package_name), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Gazebo Eddy support',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

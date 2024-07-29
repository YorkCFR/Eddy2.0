from setuptools import find_packages, setup

package_name = 'eddy_lowlevel'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Low level interface to Eddy2.0 hardware',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_node = eddy_lowlevel.thruster_node:main'
        ],
    },
)

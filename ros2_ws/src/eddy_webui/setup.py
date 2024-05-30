from setuptools import find_packages, setup

package_name = 'eddy_webui'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='Provide web-based support to introspect and control the Eddy robots',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_ui = eddy_webui.web_ui_node:main'
        ],
    },
)

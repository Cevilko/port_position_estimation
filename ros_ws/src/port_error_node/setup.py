from setuptools import find_packages, setup

package_name = 'port_error_node'

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
    maintainer='etfrobotics',
    maintainer_email='sm220315d@student.etf.bg.ac.rs',
    description='Scores triangulated port estimates against the transform tree.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'port_error_node = port_error_node.port_error_node:main',
        ],
    },
)

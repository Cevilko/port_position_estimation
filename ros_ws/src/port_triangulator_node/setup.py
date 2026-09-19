from setuptools import find_packages, setup

package_name = 'port_triangulator_node'

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
    description='Triangulates SFP port positions from multi-camera YOLO detections.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    # As with yolo_detector_node, `ros2 run` gives this a /usr/bin/python3
    # shebang. It does not need torch, but it is launched alongside the
    # detector, so ./run.sh triangulate keeps both on one interpreter.
    entry_points={
        'console_scripts': [
            'port_triangulator_node = port_triangulator_node.port_triangulator_node:main',
        ],
    },
)

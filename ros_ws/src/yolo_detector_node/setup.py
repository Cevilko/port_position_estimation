from setuptools import find_packages, setup

package_name = 'yolo_detector_node'

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
    description='Runs the trained SFP port detector on live camera topics.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # NOTE: this console script gets a /usr/bin/python3 shebang, which has no
    # torch, so `ros2 run yolo_detector_node yolo_detector_node` fails. Launch
    # it with `./run.sh detect`, which uses the venv interpreter instead.
    entry_points={
        'console_scripts': [
            'yolo_detector_node = yolo_detector_node.yolo_detector_node:main',
        ],
    },
)

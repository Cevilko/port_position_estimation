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
    # Deliberately NOT a console_scripts entry point. setuptools would give it
    # the shebang of the interpreter colcon built with (/usr/bin/python3),
    # which has no torch. scripts/ installs a wrapper into the same slot that
    # execs the venv interpreter instead, so `ros2 run` works.
    scripts=['scripts/yolo_detector_node'],
)

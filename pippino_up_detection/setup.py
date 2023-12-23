from setuptools import find_packages, setup

package_name = 'pippino_up_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michele',
    maintainer_email='michele.demarchi.85@gmail.com',
    description='This package monitors the pippino ros2 system bringup and notifies with MQTT when all the nodes are up.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pippino_up_detection_exe = pippino_up_detection.pippino_up_detection:main',
        ],
    },
)

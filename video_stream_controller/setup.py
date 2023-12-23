from setuptools import setup

package_name = 'video_stream_controller'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'video_stream_controller_exe = video_stream_controller.video_stream_controller:main',
        ],
    },
)


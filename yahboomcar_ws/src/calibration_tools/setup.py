from setuptools import setup

package_name = 'calibration_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='andy@example.com',
    description='Calibration tools for camera, LiDAR, IMU',
    license='MIT',
    entry_points={
        'console_scripts': [
            'calibration_camera = calibration_tools.calibration_camera:main',
        ],
    },
)


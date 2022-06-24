from setuptools import setup

package_name = 'emmanuel'

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
    maintainer='Alex',
    maintainer_email='alexandru.gumaniuc@student.nhlstenden.com',
    description='Navigation robot NHL Stenden Emmen',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = emmanuel.motor:main',
            'servo_controller = emmanuel.servo_motor:main',
            'speaker_controller = emmanuel.speaker:main',
            'camera_stream = emmanuel.camera:main',
            'server_goal_listener = emmanuel.server_listener:main',
            'imu_controller = emmanuel.imu:main'
        ],
    },
)

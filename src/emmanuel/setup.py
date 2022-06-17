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
    description='Navigation robot NHL',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_run = emmanuel.motors:main',
        ],
    },
)

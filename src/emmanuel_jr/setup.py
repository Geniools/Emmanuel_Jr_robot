from setuptools import setup

package_name = 'emmanuel_jr'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geniools',
    maintainer_email='deepgeniools@gmail.com',
    description='Package for controlling a navigation robot (Emmanuel Jr) across NHL Stenden University of Applied '
                'Sciences in Emmen, The Netherlands.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

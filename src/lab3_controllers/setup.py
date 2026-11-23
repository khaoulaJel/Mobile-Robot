from glob import glob
from setuptools import setup

package_name = 'lab3_controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mobile-Robot team',
    maintainer_email='noreply@example.com',
    description='P yaw and motion controllers for Lab 3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yaw_controller = lab3_controllers.yaw_controller:main',
            'motion_controller = lab3_controllers.motion_controller:main',
            'symbolic_controller = lab3_controllers.symbolic_controller:main',
        ],
    },
)

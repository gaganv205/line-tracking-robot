from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'line_follower_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ronith',
    maintainer_email='ronith@example.com',
    description='Line following robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_follower_node = line_follower_robot.line_follower_node:main',
        ],
    },
)

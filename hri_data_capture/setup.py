from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hri_data_capture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='atyshka@oakland.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_to_bag = hri_data_capture.rgb_to_bag:main',
            'multi_rgbd_to_bag = hri_data_capture.multi_rgbd_to_bag:main',
            'hue_encode_depth = hri_data_capture.hue_encode_depth:main',
        ],
    },
)

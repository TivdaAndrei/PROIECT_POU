from setuptools import setup
import os
from glob import glob

package_name = 'virtual_pet'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei Robert',
    maintainer_email='andrei@example.com',
    description='Virtual pet that draws shapes based on hand gestures',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognizer = virtual_pet.gesture_recognizer:main',
            'shape_drawer = virtual_pet.shape_drawer:main',
            'pet_controller = virtual_pet.pet_controller:main',
            'gui_controller = virtual_pet.gui_controller:main',
        ],
    },
)

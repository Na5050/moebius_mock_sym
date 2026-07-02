from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'moebius_mock'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Явный импорт папок
        #(os.path.join('share', package_name, 'launch'), [os.path.join('launch', 'mock_debug_launch.py')]),
        #(os.path.join('share', package_name, 'urdf'), [os.path.join('urdf', 'moebius_mock.urdf')]),
        #(os.path.join('share', package_name, 'config'), [os.path.join('config', 'maze_scene.yaml')]),
        #(os.path.join('share', package_name, 'config'), [os.path.join('config', 'hall.yaml')]),
        #(os.path.join('share', package_name, 'config'), [os.path.join('config', 'obstacle.yaml')]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'textures'), glob('textures/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@test.com',
    description='Mock simulator for Moebius robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Объявляем наш исполняемый python-файл как ROS2-ноду:
            'mock_moebius_maze.py = moebius_mock.mock_moebius_maze:main'
        ],
    },
)

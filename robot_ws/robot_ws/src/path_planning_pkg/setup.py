from setuptools import setup
import os
from glob import glob

package_name = 'path_planning_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.nodes',package_name + '.plugins'],  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
         (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.*'))),
        (os.path.join('share', package_name, 'models', 'meshes'),
            glob(os.path.join('models', 'meshes', '*'))),
        (os.path.join('share', package_name, 'models', 'materials', 'textures'),
            glob(os.path.join('models', 'materials', 'textures', '*'))),
        (os.path.join('share', package_name, 'models', 'thumbnails'),
            glob(os.path.join('models', 'thumbnails', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='gmohanasundaram17@gmail.com',
    description='rrt based naviagtion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_planner = path_planning_pkg.plugins.rrt_planner:main',
            'fixed_start_goal_navigator = path_planning_pkg.nodes.fixed_start_goal_navigator:main',
        ],
    },
)

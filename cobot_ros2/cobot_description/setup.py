from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cobot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch')),
        (os.path.join('share',package_name,'rviz'),glob('rviz/*.rviz')),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*.*')),
        (os.path.join('share', package_name, 'urdf/mech'), glob('urdf/mech/*')),
    	(os.path.join('share', package_name, 'urdf/robots'), glob('urdf/robots/*')),
        (os.path.join('share',package_name,'meshes'),glob('meshes/*.*')),
        (os.path.join('share', package_name, 'meshes/g_shape_base_v2_0/visual'), glob('meshes/g_shape_base_v2_0/visual/*')),
        (os.path.join('share', package_name, 'meshes/adaptive_gripper/visual'), glob('meshes/adaptive_gripper/visual/*')),
        (os.path.join('share', package_name, 'meshes/mycobot_280/visual'), glob('meshes/mycobot_280/visual/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaansnowman',
    maintainer_email='ahmet1kaanirgin@gmail.com',
    description='TODO: Package description',
    license='BDS-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

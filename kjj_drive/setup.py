from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'kjj_drive'
packageSharPath = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(packageSharPath, 'launch'), glob('launch/*.launch.py')),
        (os.path.join(packageSharPath, 'urdf'), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oem',
    maintainer_email='oem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kjj_drive = kjj_drive.kjj_drive:main',

        ],
    },
)

import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'local_usbcam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theseus',
    maintainer_email='38895035+cbares@users.noreply.github.com',
    description='local_usbcam package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'local_usbcam_start = local_usbcam.start_cam:main'
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shared_objects'


setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    zip_safe=True,
    packages=['shared_objects'],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    package_dir={'': 'src'},
    maintainer='Abdullah Celik',
    maintainer_email='gerkey@example.com',
    description='Shared_objects',
    license='BSD',
    entry_points={
    'console_scripts': [
    	'utils_stop = shared_objects.utils_stop:main',
        'utils_path = shared_objects.utils_path:main',
        'Motor = shared_objects.Motor:main',
        'ROS_utils = shared_objects.ROS_utils:main',
        'utils_model = shared_objects.utils_model:main',
    ],
},
)

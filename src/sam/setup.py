from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', 'params.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ardimou',
    maintainer_email='ardimou@physics.auth.gr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['seg_points_srv = sam.point_seg_serv:main',
                            'seg_points_client = sam.point_seg_client:main',
                            'seg_boxes_srv = sam.bboxes_seg_serv:main',
                            'seg_boxes_client = sam.bboxes_seg_client:main',
                            ],
    },
)

import os
from glob import glob
from setuptools import setup

package_name = 'graspnet_service'

setup(
    name=package_name,
    version='0.19.1',
    packages=[package_name, package_name + '/models', package_name + '/utils', package_name + '/knn', package_name + '/pnet2'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'weights'), glob('weights/*')),
         (os.path.join('share', package_name, 'data'), glob('data/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Sotiris Malasiotis',
    author_email='malasiot@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Graspnet baseline ROS2 service',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = graspnet_service.service:main'
          
        ],
    },
)
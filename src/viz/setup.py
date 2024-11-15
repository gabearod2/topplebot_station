from setuptools import setup
from glob import glob

package_name = 'viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='your.email@example.com',
    description='Visualization package for ToppleBot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'tb_tf_broadcaster = viz.tb_tf_broadcaster:main',
        ],
    },
)

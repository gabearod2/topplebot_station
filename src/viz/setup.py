from setuptools import find_packages, setup

package_name = 'viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='ToppleBot RViz visualization support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'imu_to_tf_broadcaster = viz.topplebot_tf_broadcaster:main',
    ],
}
,
)
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'delay_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lliw',
    maintainer_email='w1359066726@gmail.com',
    description='TODO: simulate time delay during communication in ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'my_delayer = delay_simulation.variable_delay_listener:main', # 节点名=包名.脚本名:main
                'talker = delay_simulation.normal_talker:main'
        ],
},
)

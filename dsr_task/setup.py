from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dsr_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),  # package.xml 파일 매핑
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # launch 파일 매핑
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mk',
    maintainer_email='kmingi98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tool_changer = dsr_task.tool_changer:main',
            'tool_changer2 = dsr_task.tool_changer2:main',
            'image_capture = dsr_task.image_capture:main',
            'homming = dsr_task.homming:main',
            'gripper_control = dsr_task.gripper_control:main',
            'move_robot = dsr_task.move_robot:main',
            'trans = dsr_task.trans:main',

        ],
    },
)

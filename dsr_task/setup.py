from setuptools import find_packages, setup

package_name = 'dsr_task'

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

        ],
    },
)

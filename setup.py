from setuptools import setup
from glob import glob

package_name = 'dynamic_reconfigure_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petrvancjr@gmail.com',
    description='',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_reconfigure_ros2 = dynamic_reconfigure_ros2.dynamic_reconfigure:main',
        ],
    },
)





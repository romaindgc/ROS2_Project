from setuptools import find_packages, setup

package_name = 'robotFollower'

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
    maintainer='rom1',
    maintainer_email='romain.degrace@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotFollower = robotFollower.robotFollower:main',
            'robotLeader = robotFollower.leaderPath:main',
            'centered_cmd = robotFollower.centered_cmd:main',
        ],
    },
)

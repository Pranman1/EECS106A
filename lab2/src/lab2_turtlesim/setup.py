from setuptools import find_packages, setup

package_name = 'lab2_turtlesim'

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
    maintainer='pranav',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_controller = lab2_turtlesim.controller:main',
            'patrol_server = lab2_turtlesim.patrol_server:main',
            'patrol_client = lab2_turtlesim.patrol_client:main',
        ],
    },
)

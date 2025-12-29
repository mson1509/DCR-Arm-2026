from setuptools import find_packages, setup

package_name = 'motor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_arm.py']),
    ],
    install_requires=['setuptools',
                    'ikpy'],
    zip_safe=True,
    maintainer='bonnguyen',
    maintainer_email='minhsonnguyen1509@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start_motor = motor_node.main:main',
            'start_controller = motor_node.controller:main',
        ],
    },
)

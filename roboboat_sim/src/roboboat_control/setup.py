from glob import glob

from setuptools import find_packages, setup

package_name = 'roboboat_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Doug Johnson',
    maintainer_email='doug@douglasjohnson.org',
    description='Surrogate 3-DOF dynamics and thrust allocation for the RoboBoat sim.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'surrogate_dynamics = roboboat_control.surrogate_dynamics_node:main',
            'thruster_markers = roboboat_control.thruster_markers_node:main',
        ],
    },
)

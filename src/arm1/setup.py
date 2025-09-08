from setuptools import find_packages, setup
from glob import glob
package_name = 'arm1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob("config/*.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lzy',
    maintainer_email='lzy@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_arm_topic_control = arm1.test_arm_topic_control:main",
            "test_hand_control = arm1.test_hand_control:main"
        ],
    },
)

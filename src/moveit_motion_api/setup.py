from setuptools import find_packages, setup

package_name = 'moveit_motion_api'

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
    maintainer='lzy',
    maintainer_email='lzy@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test1_node = moveit_motion_api.test1:main',
            # 'custom_clock_bridge = moveit_motion_api.custom_clock_bridge:main',
        ],
    },
)

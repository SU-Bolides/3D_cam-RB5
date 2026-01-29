from setuptools import setup

package_name = 'rb5_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='RB5 navigation package: camera + lidar fusion and obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = rb5_obstacle_avoidance.camera_avoidance:main',
            'fusion = rb5_obstacle_avoidance.camera_lidar_avoidance:main',
            'fusion_emergency = rb5_obstacle_avoidance.fusion_emergency_stop:main',
        ],
    },
)

from setuptools import setup
package_name = 'ros2_automower_localization'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Toro',
    maintainer_email='N/A',
    description='Localization package for ros2 automower',
    license='MIT',
    tests_require=['pytest'],
    entry_points = {
    'console_scripts': [
        'imu = ros2_automower_localization.imu:main'
    ]
}
)

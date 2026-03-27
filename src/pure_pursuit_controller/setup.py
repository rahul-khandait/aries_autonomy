from setuptools import setup

package_name = 'pure_pursuit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahul',
    maintainer_email='rahul@todo.todo',
    description='Pure Pursuit controller for rover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit_controller.pure_pursuit_node:main',
            'path_generator = pure_pursuit_controller.path_generator:main',
            'imu_odom_validator = pure_pursuit_controller.imu_odom_validator:main',
        ],
    },
)

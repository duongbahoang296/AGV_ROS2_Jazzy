from setuptools import setup

package_name = 'agv_motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoang',
    maintainer_email='xxx@gmail.com',
    description='AGV motor driver',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_driver = agv_motor_driver.motor_driver:main',
        ],
    },
)
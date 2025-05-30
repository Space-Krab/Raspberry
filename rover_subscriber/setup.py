from setuptools import find_packages, setup

package_name = 'rover_subscriber'

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
    maintainer='xplore',
    maintainer_email='leonard.lemaire@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'live_subscriber = rover_subscriber.new_subscriber:main',
            'subscriber = rover_subscriber.subscriber:main',
            'camera = rover_subscriber.camera_pub:main',
        ],
    },
)

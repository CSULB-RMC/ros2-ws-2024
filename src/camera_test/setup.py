from setuptools import find_packages, setup

package_name = 'camera_test'

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
    maintainer='leopan42',
    maintainer_email='leo.long.pan@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = camera_test.cam_sub:main',
            'listener2 = camera_test.cam_sub_2:main',
            'talker = camera_test.cam_pub:main',
            'talker2 = camera_test.cam_pub_2:main',
        ],
    },
)

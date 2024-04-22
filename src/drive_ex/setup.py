from setuptools import find_packages, setup

package_name = 'drive_ex'

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
    maintainer='leho42',
    maintainer_email='leo.long.pan@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drivetrain_ex = drive_ex.drivetrain_ex_sub:main',
            'drivetrain_cb = drive_ex.drivetrain_mini_sub:main',
        ],
    },
)

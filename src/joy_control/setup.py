from setuptools import find_packages, setup

package_name = 'joy_control'

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
    maintainer='porterclev',
    maintainer_email='porterclev@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_ex = joy_control.joy_ex_pub:main',
            'control_cb = joy_control.joy_cb_pub:main'
        ],
    },
)

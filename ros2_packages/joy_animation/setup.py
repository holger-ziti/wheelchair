from setuptools import setup

package_name = 'joy_animation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='holger',
    maintainer_email='holger.dieterich@ziti.uni-heidelberg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'animation_1 = joy_animation.joy_animation_1:main',
            'animation_2 = joy_animation.joy_animation_2:main',
        ],
    },
)

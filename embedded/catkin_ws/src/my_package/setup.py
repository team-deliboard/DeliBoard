from setuptools import setup

package_name = 'my_package'

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
    maintainer='SSAFY',
    maintainer_email='SSAFY@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'talker = my_package.publisher_member_function:main',
            'listener = my_package.subscriber_member_function:main',
            'communication = my_package.communication:main',
            'odom = my_package.odom:main',
            'path = my_package.path:main',
            'carrot = my_package.carrot:main',
            'collide = my_package.collide:main',
            'following = my_package.following:main',
            'makePath = my_package.makePath:main',
            'MoveRobot = my_package.MoveRobot:main',
            'carry=my_package.carry:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'piper_x_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiangqi',
    maintainer_email='jiangqi@todo.todo',
    description='Piper X MoveIt2 planning/execution demo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_demo = piper_x_app.moveit_demo:main',
        ],
    },
)

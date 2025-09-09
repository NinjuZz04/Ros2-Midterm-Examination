from setuptools import find_packages, setup

package_name = 'mid_action_node'

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
    maintainer='ninju',
    maintainer_email='ninju@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'rotate_action_server = mid_action_node.rotate_action_server:main',
             'rotate_action_client = mid_action_node.rotate_action_client:main'
        ],
    },
)

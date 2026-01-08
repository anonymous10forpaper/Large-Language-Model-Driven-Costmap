from setuptools import find_packages, setup

package_name = 'policy_bridge'

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
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_bridge = policy_bridge.policybridge:main',
            'policy_bridge_multi = policy_bridge.policybridge_multi:main',
            'nl_command_sender = policy_bridge.nl_command_sender:main',
            'event_sender = policy_bridge.event_sender:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'picarx_system_controller'

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
    maintainer='picarx',
    maintainer_email='picarx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = picarx_system_controller.system_control_node_test:main",
            "controller_node = picarx_system_controller.picarx_controller_node:main"
        ],
    },
)

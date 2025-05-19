from setuptools import setup

package_name = 'robus_vehicle_map_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vaithiyanathan Alagar',
    maintainer_email='vaithish1109@gmail.com',
    description='Visualize ego and other vehicles, road work vehicle on a model city map using RViz2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robus_vehicle_map_node = robus_vehicle_map_pkg.robus_vehicle_map_node:main',
        ],
    },
)


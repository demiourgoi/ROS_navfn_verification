from setuptools import setup

package_name = 'pubsub_maude_ros'

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
    maintainer='Enrique Martin',
    maintainer_email='emartinm@ucm.es',
    description='Ejemplo de nodos ROS2 invocando a Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'mauder_talker = pubsub_maude_ros.publisher:main',
                'mauder_listener = pubsub_maude_ros.subscriber:main',
        ],
    },
)

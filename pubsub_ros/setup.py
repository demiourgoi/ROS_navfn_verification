from setuptools import setup

package_name = 'pubsub_ros'

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
                'talker = pubsub_ros.publisher:main',
                'listener = pubsub_ros.subscriber:main',
        ],
    },
)

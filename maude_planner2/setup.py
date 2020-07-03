from setuptools import setup

package_name = 'maude_planner'

setup(
    name=package_name,
    version='0.1.0',
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
    description='A planner for ROS robots written in Maude',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'maude_planner = maude_planner.planner_action_server:main',
        ],
    },
)

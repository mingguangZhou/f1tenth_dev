from glob import glob
from setuptools import setup

package_name = 'centerline_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/centerline_output', glob('centerline_output/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='support@openai.com',
    description='ROS 2 tools for publishing offline-generated centerlines.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centerline_publisher = centerline_tools.centerline_publisher_node:main',
        ],
    },
)

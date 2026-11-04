from glob import glob
from pathlib import Path
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
        # Portable curated selections refer to source-root maps; keep that layout.
        ('share/' + package_name, glob('*.yaml') + glob('*.pgm') + glob('*.png')),
        *[(str(Path('share') / package_name / directory),
           [str(p) for p in sorted(Path(directory).iterdir()) if p.is_file()])
          for directory in sorted({str(p.parent) for p in Path('tracks').rglob('*') if p.is_file()})],
        ('share/' + package_name + '/centerline_output', [p for p in glob('centerline_output/*') if Path(p).is_file()]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='support@openai.com',
    description='ROS 2 tools for publishing offline-generated centerlines and racelines.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'global_raceline_optimizer = '
                'centerline_tools.global_raceline_optimizer:main'
            ),
            'raceline_publisher = centerline_tools.centerline_publisher_node:main',
        ],
    },
)

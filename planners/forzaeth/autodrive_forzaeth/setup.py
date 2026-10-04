from glob import glob
from setuptools import setup

package_name = 'autodrive_forzaeth'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'fast_bridge = autodrive_forzaeth.fast_bridge:main',
            'adapter = autodrive_forzaeth.adapter:main',
            'mapper = autodrive_forzaeth.mapper:main',
            'run_logger = autodrive_forzaeth.run_logger:main',
        ],
    },
)

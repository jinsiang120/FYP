import os
from glob import glob
from setuptools import setup

package_name = 'beta_desc'

data_files = [('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
              ('share/' + package_name, ['package.xml'])]

additional_data_files = []

def getDestinationTuple(value):
    return (os.path.join('share', package_name, os.path.dirname(value)), [value])

for pattern in ('urdf/**/*.*', 'launch/**/*.*','worlds/**/*.*'):
    additional_data_files.extend(
        list(map(getDestinationTuple, glob(pattern, recursive=True))))

data_files.extend(additional_data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinsiang120',
    maintainer_email='jinsiang120@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spawn_beta = beta_desc.spawn_beta:main',
        ],
    },
)

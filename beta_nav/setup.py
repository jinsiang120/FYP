import os
from glob import glob
from setuptools import setup


package_name = 'beta_nav'

data_files = [('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
              ('share/' + package_name, ['package.xml'])]

additional_data_files = []

def getDestinationTuple(value):
    return (os.path.join('share', package_name, os.path.dirname(value)), [value])

for pattern in ('map/**/*.*', 'launch/**/*.*','params/**/*.*','behaviour_trees/**/*.*'):
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
        'point_following = beta_nav.point_following:main',
        'robot_navigator = beta_nav.robot_navigator:main',
        'simple = beta_nav.simple:main',
        'dijkstra = beta_nav.dijkstra:main',
        'simple_formation= beta_nav.simple_formation:main',
        'simple_formation2= beta_nav.simple_formation2:main',

        ],
    },
)

from setuptools import setup

package_name = 'beta_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'launch/online_async_launch.py', 'configs/mapper_params_online_async.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinsiang120',
    maintainer_email='jinsiang120@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

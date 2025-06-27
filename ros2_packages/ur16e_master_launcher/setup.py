from setuptools import find_packages, setup

package_name = 'ur16e_master_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur_full_stack.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nordbo',
    maintainer_email='arnaud.candaele@buildwise.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

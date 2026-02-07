from setuptools import find_packages, setup

package_name = 'person_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
        'launch/people_track.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chetansai',
    maintainer_email='chetansai@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = person_tracker.detector:main',
            'tracker = person_tracker.tracker:main',
        ],
    },
)

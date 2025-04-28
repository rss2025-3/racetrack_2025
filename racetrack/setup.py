from setuptools import find_packages, setup
import glob
import os

package_name = 'racetrack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/'+package_name+"/computer_vision", glob.glob(os.path.join('racetrack/computer_vision', '*.py'))),
        ('share/racetrack/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/racetrack/launch', glob.glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='32546943+2v@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'racetrack_controller = racetrack.racetrack_controller:main',
            'line_detector = racetrack.cone_detector:main',
            'cone_sim_marker = racetrack.cone_sim_marker:main',
            'homography_transformer = racetrack.homography_transformer:main',
        ],
    },
)

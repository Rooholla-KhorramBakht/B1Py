from setuptools import setup
import os
from glob import glob

package_name = 'b1py_calib'

params_path = os.path.join('share', package_name, 'params')
param_files = []
for root, dirs, files in os.walk(params_path):
    for file in files:
        param_files.append((os.path.join('share', package_name, root), [os.path.join(root, file)]))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params','kalibr'), glob('params/kalibr/*')),
        (os.path.join('share', package_name, 'params','simplehandeye'), glob('params/simplehandeye/*')),
        (os.path.join('share', package_name, 'params','vicon2gt'), glob('params/vicon2gt/*')),
    ],
    install_requires=['setuptools', 'gtsam'],
    zip_safe=True,
    maintainer='Rooholla Khorrambakht',
    maintainer_email='rk4342@nyu.edu',
    description='A package to broadcast information stored in the calibration results files form Kalibr, SimpleHandEye, and vicon2gt as TF2 static transforms.',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calib_broadcaster = b1py_calib.calib_broadcaster:main'
        ],
    },
)

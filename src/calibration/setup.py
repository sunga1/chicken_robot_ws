from setuptools import setup
import os          
from glob import glob

package_name = 'calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # config 내의 yaml파일을 install 폴더로 복사하도록 설정
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunga',
    maintainer_email='sunga1117@naver.com',
    description='Chicken Robot Calibration Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrator = calibration.calibrator:main',
            'tf_broadcaster = calibration.tf_broadcaster:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'chicken_task_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunga',
    maintainer_email='sunga@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        # '실행할_이름 = 패키지명.파일명:함수명'
        'point_to_plan = chicken_task_planner.point_to_plan:main',
    ],
    },
)

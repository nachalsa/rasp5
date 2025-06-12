# setup.py

from setuptools import find_packages, setup

package_name = 'picar_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@todo.com',
    description='Autonomous Car Project for PicAR',
    license='Apache-2.0',
    tests_require=['pytest'],
    # ▼▼▼▼▼ 이 부분을 수정합니다 ▼▼▼▼▼
    entry_points={
        'console_scripts': [
            # 'src.' 부분을 모두 제거합니다.
            'test_runner = picar_pkg.test:main',
            'main_runner = picar_pkg.main_node:main',
        ],
    },
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
)


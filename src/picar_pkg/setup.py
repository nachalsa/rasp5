import os
from glob import glob
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
        # launch 파일이 있다면, launch 폴더를 만들어 이 줄의 주석을 푸세요.
        # (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        # config 파일이 있다면, 이 줄의 주석을 푸세요.
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsh',
    maintainer_email='your_email@example.com', # 이메일 수정 필요
    description='Picar project ROS 2 package',
    license='Apache License 2.0', # 라이선스에 맞게 수정
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 예시: '실행파일이름 = 파이썬패키지경로.파일이름:main함수'
            'main_node = picar_pkg.main_node:main',
        ],
    },
)

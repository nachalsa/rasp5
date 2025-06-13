from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'picar_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']), # 'test' 폴더는 패키지에 포함 안 함

    # ★★★ 이 부분이 중요합니다 ★★★
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일을 위한 설정 (launch 폴더를 만든다면)
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        # config 파일을 위한 설정 (config 폴더가 있다면)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsh',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',

    # ★★★ 테스트를 위한 설정 ★★★
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 예시: '실행명 = 패키지명.파일이름:main'
            'main_node = picar_pkg.main_node:main',
        ],
    },

from setuptools import find_packages, setup

package_name = 'picar_pkg' # 1. 패키지 이름 (package.xml과 동일해야 함)

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
    entry_points={
        'console_scripts': [
            # '실행명령어 = 경로.파일명:main함수' 형식
            # 예: ros2 run picar_pkg test_runner
            'test_runner = src.picar_pkg.test:main',
            'main_runner = src.picar_pkg.main_node:main',
        ],
    },

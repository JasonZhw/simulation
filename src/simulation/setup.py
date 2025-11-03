# src/simulation/setup.py
from setuptools import setup, find_packages
import os

package_name = 'simulation'

# 1) 递归打包 conde_world 模型资源
data_files = []
model_root = os.path.join('resource', 'models', 'conde_world')
for dirpath, _, filenames in os.walk(model_root):
    if filenames:
        install_dir = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_dir, files))

# 2) 其他资源 + ament 索引 + package.xml
data_files += [
    # 标准包标记（必须：文件名=包名 simulation；放在你的仓库 resource/simulation）
    ('share/ament_index/resource_index/packages', ['resource/simulation']),
    # 安装 package.xml
    (f'share/{package_name}', ['package.xml']),
    # 安装 launch
    (f'share/{package_name}/launch', ['launch/gazebo_launch.py']),
    # 安装 world/xacro 等
    (f'share/{package_name}/resource', [
        'resource/conde_world.world',
        'resource/lane_track.world',
        'resource/track.world',
        'resource/track.dae',
        'resource/car.xacro',
    ]),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Integrated Gazebo sim with perception and control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_following = simulation.lane_following:main',
            'viz = simulation.visualization:main',
        ],
    },
)

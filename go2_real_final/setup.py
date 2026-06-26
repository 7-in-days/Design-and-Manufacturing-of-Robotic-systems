from setuptools import find_packages, setup

package_name = 'go2_final'

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
    maintainer='joo',
    maintainer_email='badukjoo@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go2_map1 = go2_final.go2_map1_controller:main',
            'go2_map2 = go2_final.go2_map2_controller:main',
            'occupancy_map_rviz_debug = go2_final.occupancy_map_rviz_debug:main',
        ],
    },
)

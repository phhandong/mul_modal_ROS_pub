from setuptools import setup


package_name = 'radar'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/radar'],
        ),
        (
            'share/radar/launch',
            [
                'launch/quantum.launch.py',
                'launch/ars408.launch.py',
                'launch/ars408_playback.launch.py',
            ],
        ),
        ('share/radar/rviz', ['rviz/ars408.rviz']),
        ('share/radar', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'quantum_radar=radar.quantum:main',
            'ars408_radar=radar.ars408:main',
            'ars408_bag_recorder=radar.bag_recorder:main',
            'ars408_speed_filter=radar.speed_filter:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'robot_voice_control'

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
    maintainer='doanh',
    maintainer_email='doanh762003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_to_nav = robot_voice_control.voice_to_nav:main',
            'audio_publisher = robot_voice_control.audio_publisher:main',
            'speech_to_cmd = robot_voice_control.speech_to_cmd:main',
        ],
    },
)

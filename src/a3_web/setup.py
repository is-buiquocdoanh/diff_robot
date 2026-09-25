import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'a3_web'


def static_files():
    """Cài toàn bộ thư mục static/ (kể cả vendor/) vào share/a3_web/static."""
    out = []
    for root, _, files in os.walk('static'):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + static_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='roboticsvn.ai@gmail.com',
    description='Web local điều khiển robot A3: quét/lưu bản đồ, waypoint, điều hướng Nav2, teleop.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = a3_web.server:main',
            'fake_robot = a3_web.tools.fake_robot:main',
            'fake_stack = a3_web.tools.fake_stack:main',
        ],
    },
)

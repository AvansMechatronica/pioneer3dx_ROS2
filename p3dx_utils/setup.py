from setuptools import find_packages, setup

package_name = 'p3dx_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='gerard',
    maintainer_email='GerardAnneHarkema@gmail.com',
    description='Utilities for Pioneer 3DX robot including status monitor GUI',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'p3dx_status_monitor = p3dx_utils.status_monitor_gui:main',
        ],
    },
)

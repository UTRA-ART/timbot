from setuptools import find_packages, setup

package_name = 'timbot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5>=5.15.0'],
    zip_safe=True,
    maintainer='y3egan',
    maintainer_email='reagan.hu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'timbot_gui = timbot_gui.gui_node:main',
    ],
},
)

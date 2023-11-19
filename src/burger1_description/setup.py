from setuptools import find_packages, setup

package_name = 'burger1_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/resource', ['resource/burger1.urdf.xacro']),
        (f'share/{package_name}/resource', ['resource/base_link.xacro']),
        (f'share/{package_name}/launch', ['launch/gazebo_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergio',
    maintainer_email='sergiorincon50@gmail.com',
    description='Robot description package for Burger1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Puedes agregar scripts de consola aquí si los necesitas en el futuro
        ],
    },
)

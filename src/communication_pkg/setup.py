from setuptools import find_packages, setup

package_name = 'communication_pkg'

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
    maintainer='vitor-lucas-fujita-fel-cio',
    maintainer_email='vlucasff@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processamento_body38 = communication_pkg.body38_processamento_node:main',
            'processamento_body34 = communication_pkg.body34_processamento_node:main',
            'processamento_node = communication_pkg.processamento_node:main',
        ],
    },
)
